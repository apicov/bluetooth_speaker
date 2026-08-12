/*
 * The playback timeline: holding the first sample for its instant, measuring
 * phase at each packet boundary, splicing at a track change, and the marker.
 *
 * Split out of main.c on 2026-08-12; the bodies are unchanged.
 */
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_app_desc.h"
#include "nvs_flash.h"

#include "audio_out.h"
#include "sync_proto.h"
#include "sbc_link.h"
#include "sbc_decoder.h"
#include "visualiser.h"
#include "wifi_log.h"

#include "sat.h"

void play_task(void *arg)
{
    (void)arg;
    static uint8_t chunk[AUDIO_CHUNK_BYTES];

    while (1) {
        if (stream_start_local == 0) {
            /* Parked, by whichever route -- resync, underrun or a changed
             * timeline. The flag has been served either way, and clearing it
             * here rather than only where it is consumed is what lets the
             * anchor path below wait on it without being able to deadlock. */
            resync_request = false;
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        /* Hold the first sample until its moment arrives. After this, I2S paces
         * everything: i2s_channel_write blocks once the DMA buffers are full. */
        int64_t wait = stream_start_local - esp_timer_get_time();
        if (wait > 2000) {
            vTaskDelay(pdMS_TO_TICKS((wait - 2000) / 1000));
        }
        while (esp_timer_get_time() < stream_start_local) {
            /* spin the last stretch, as in the M4 blink task */
        }
        int64_t actual_master = esp_timer_get_time() + stream_offset;
        int64_t sched_master = stream_start_local + stream_offset;
        ESP_LOGI(TAG, "playback started: scheduled %lld, actual %lld (%+lld us) [master]",
                 sched_master, actual_master, actual_master - sched_master);
        /*
         * samples_played counts from the first sample played, which is the
         * first sample queued after the ring was reset on anchoring. The two
         * counters therefore share an origin -- do NOT reset samples_in here.
         * It has legitimately been counting the audio buffered during the wait,
         * and zeroing it shifts every marker by the buffer depth.
         */
        int32_t samples_played = 0;
        /* Every reading in it was measured against the stream this anchor
         * replaces, so none of them describes where this unit now is. */
        sync_phase_reset(&phase_hist);
        /*
         * Measure the DMA prefill at every playback START, not only after a
         * retune. The channel has been draining while this task was parked, so
         * it is empty here for exactly the same reason it is empty after a
         * disable -- and i2s_channel_write() does not block until the
         * descriptors are full, so the first writes return at memory speed and
         * samples_played advances by the whole DMA depth against a wrote_at
         * that has barely moved. Every phase reading dated from that window is
         * measured against a reference the DAC is not pacing.
         *
         * This is the same mechanism the `retuning` park exists to prevent, and
         * clock-sync.md records it costing +42, +43 and +50 ms there before the
         * park existed. Nothing guards it here.
         *
         * It is very likely the "-42 ms (hub), -26 ms (satellite)" startup phase
         * in clock-sync.md §8 -- a 16 ms difference between two units that then
         * takes ~45 s to walk off, on a cold start, on both units at once. On a
         * reconnect only this unit restarts, against a hub already servoed to
         * zero, which is a different and much easier situation.
         *
         * MEASUREMENT ONLY: the REFILL line reports, nothing withholds. Whether
         * to withhold is the next question and this is what sizes it.
         */
        s_refill_active = true;
        s_refill_frames = 0;
        /* When the DAC last accepted a chunk -- what the phase reading is dated
         * against. See the hub's copy for why it is not a clock read taken in
         * the phase loop. Seeded so the first pass has a sane value. */
        int64_t wrote_at = esp_timer_get_time();

        while (1) {
            if (retuning) {
                /* Do not pull from the ring while the channel is down -- the
                 * writes would return instantly and drain it. */
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
            /*
             * The receive task found a gap too large to fill and wants a clean
             * restart -- see GAP_RESYNC_MS. Leave by the same door as an
             * underrun, so the anchor path owns the ring reset rather than
             * racing this loop for it. Noticed within one chunk (~5.8 ms),
             * comfortably inside the ~20 ms until the next packet arrives.
             *
             * Not logged here. This task is the audio path, and putting an
             * ESP_LOGW on it is the mistake the RX counters exist to undo;
             * drift_task narrates n_gap_resyncs within 5 s instead.
             */
            if (resync_request) {
                resync_request = false;
                stream_start_local = 0;
                phase_valid = false;
                phase_stepped = true;
                break;
            }
            hw_play = uxTaskGetStackHighWaterMark(NULL);   /* only valid in-task */
            size_t got = xStreamBufferReceive(ring, chunk, sizeof(chunk), pdMS_TO_TICKS(500));
            if (got == 0) {
                n_underruns++;
                ESP_LOGW(TAG, "underrun, waiting for a new stream");
                stream_start_local = 0;
                break;
            }
            if (got < sizeof(chunk)) {
                memset(chunk + got, 0, sizeof(chunk) - got);
                n_short_reads++;
                n_short_frames += (uint32_t)((sizeof(chunk) - got)
                                             / (AUDIO_CHANNELS * sizeof(int16_t)));
            }
            /* Before measuring anything against the master clock, make sure the
             * conversion still describes it. */
            track_offset();

            /* Has playback reached a recorded packet boundary? If so, compare
             * now against when that sample was due. */
            bool timeline_changed = false;
            while (phase_tail != phase_head && samples_played >= phase_q[phase_tail].pos) {
                int64_t due = phase_q[phase_tail].play_at;
                /*
                 * Dated by where the crossing HAPPENED, not where the loop
                 * noticed it. samples_played moves AUDIO_FRAMES at a time --
                 * 5.8 ms at 44.1 kHz -- so the crossing is up to a chunk in the
                 * past by an amount that depends on where pos falls on the
                 * chunk grid, which is uncorrelated noise straight into the
                 * servo's only input. The overshoot is known, and writes are
                 * paced by the DAC, so the correction is exact rather than a
                 * filter. See the hub's copy for what this cost there.
                 */
                int32_t overshoot = samples_played - phase_q[phase_tail].pos;
                /* Capped at one chunk: a splice steps samples_played by up to
                 * MAX_SPLICE_MS at once and those frames were skipped rather
                 * than played, so anything the jump carried us past cannot be
                 * dated this way. Same guard as the hub. */
                if (overshoot > (int32_t)AUDIO_FRAMES) {
                    overshoot = (int32_t)AUDIO_FRAMES;
                }
                /* Dated from when the DAC last took a chunk, not from a clock
                 * read here -- the write is the only DAC-paced event in this
                 * loop, and everything between it and this line is unpaced.
                 * See the hub's copy, which is where that mattered. */
                /* The crossing instant on THIS unit's clock. now_master is the
                 * same instant converted; the local form is what dates the
                 * crossing against a retune, which is a local event. */
                int64_t crossed_at = wrote_at
                                   - (int64_t)overshoot * 1000000 / stream_rate;
                int64_t now_master = crossed_at + stream_offset;
                int64_t err = now_master - due;
                /*
                 * Seconds of error is not drift and not jitter. It means the
                 * stamps are being issued against a different clock origin than
                 * the one playback anchored to -- the hub rebooted or was
                 * reflashed while this unit kept playing -- and no servo can
                 * correct that, because there is nothing wrong with the rate.
                 *
                 * It used to be cast straight into an int32: an hour of error
                 * wrapped to -699 seconds, the smoothing overflowed on top of
                 * it, and the servo asked for a 4.29 GHz sample rate, which
                 * aborted the board. Re-anchor instead, which is the one action
                 * that actually fixes it -- it re-reads the offset against the
                 * clock the hub is really using now.
                 */
                if (err > PHASE_INSANE_US || err < -PHASE_INSANE_US) {
                    ESP_LOGE(TAG, "phase %lld us -- not the timeline we anchored to, "
                                  "re-anchoring", err);
                    timeline_changed = true;
                    break;
                }
                /* The first reading after a retune is a transient -- logged,
                 * then thrown away rather than handed to the servo. See the
                 * hub's copy for what the outage figure actually covers and
                 * for the bench numbers that came off this unit. */
                const int64_t since_retune = retune_done_at
                                           ? crossed_at - retune_done_at : -1;
                if (retune_watch) {
                    retune_watch = false;
                    ESP_LOGW(TAG, "RETUNE COST: phase %+ld -> %+lld us (net %+lld), "
                                  "channel was down %lld us -- withheld from the "
                                  "servo, crossed %lld us after the retune",
                             (long)retune_phase_before, err, err - retune_phase_before,
                             retune_outage_us, since_retune);
                } else {
                    /*
                     * Narrated but NOT withheld -- these reach the servo exactly
                     * as they did before, so behaviour is unchanged and only the
                     * log says more. Whether they SHOULD be withheld is the
                     * question these lines exist to answer.
                     */
                    if (retune_tail_left) {
                        retune_tail_left--;
                        ESP_LOGW(TAG, "RETUNE TAIL: phase %+lld us at %lld us after "
                                      "the retune (net %+lld from before it)",
                                 err, since_retune, err - retune_phase_before);
                    }
                    phase_err_us = (int32_t)err;
                    phase_valid = true;
                    sync_phase_push(&phase_hist, (int32_t)err);
                }
                phase_tail = (phase_tail + 1) % PHASE_Q_LEN;
            }
            if (timeline_changed) {
                /* Same exit as an underrun: handle_audio() re-anchors on the
                 * next packet, which re-seeds stream_offset from a current
                 * estimate rather than one describing a hub that no longer
                 * exists. Drop the smoothing with it -- every sample in it was
                 * measured against the old origin. */
                stream_start_local = 0;
                phase_valid = false;
                phase_stepped = true;
                break;
            }

            /*
             * Track boundary reached: snap phase to zero rather than letting the
             * servo walk it off over ~45 s. Skipping or inserting audio is
             * inaudible here and nowhere else.
             */
            int32_t rp = restart_pos;
            if (rp >= 0 && samples_played >= rp) {
                restart_pos = -1;
                int32_t max_frames = (int32_t)stream_rate * MAX_SPLICE_MS / 1000;
                /* Same guard as the hub: phase_err_us survives a re-anchor, so
                 * a boundary reached before the first measurement of the new
                 * stream would splice on a number describing the old one. */
                int32_t adj = phase_valid
                    ? (int32_t)((int64_t)phase_err_us * stream_rate / 1000000) : 0;
                if (adj > max_frames)  adj = max_frames;
                if (adj < -max_frames) adj = -max_frames;
                int32_t applied = 0;      /* what the splice actually moved */

                /*
                 * SHADOW: the correction the median of the last few readings
                 * would have asked for, clamped identically so it subtracts
                 * meaningfully against the hub's. Reported to the hub, acted on
                 * by nothing -- the splice above still runs on phase_err_us.
                 * Same computation as the hub's copy, deliberately.
                 */
                int32_t med_us = 0;
                if (phase_valid && sync_phase_median(&phase_hist, &med_us)) {
                    int32_t med_adj = (int32_t)((int64_t)med_us * stream_rate / 1000000);
                    if (med_adj > max_frames)  med_adj = max_frames;
                    if (med_adj < -max_frames) med_adj = -max_frames;
                    splice_report_med = (int32_t)((int64_t)med_adj * 1000000 / stream_rate);
                } else {
                    splice_report_med = 0;
                }

                if (adj > 0) {
                    /* Late: discard input so playback jumps forward in content.
                     * samples_played tracks input consumed, so it advances too.
                     *
                     * Into its own buffer, not into chunk: chunk is holding the
                     * audio read at the top of this pass, which has not been
                     * played yet. Discarding into it threw that away and played
                     * the tail of the skipped region in its place -- 5.8 ms of
                     * the wrong audio at every boundary. Nothing drifted, since
                     * both reads are counted, but it is not what the splice is
                     * supposed to do. */
                    static uint8_t discard[AUDIO_CHUNK_BYTES];
                    int32_t left = adj;
                    while (left > 0) {
                        size_t want = (size_t)(left > (int32_t)AUDIO_FRAMES ? AUDIO_FRAMES : left)
                                      * AUDIO_CHANNELS * sizeof(int16_t);
                        size_t got2 = xStreamBufferReceive(ring, discard, want, 0);
                        if (got2 == 0) break;
                        left -= got2 / (AUDIO_CHANNELS * sizeof(int16_t));
                    }
                    samples_played += (adj - left);
                    applied = adj - left;
                    ESP_LOGW(TAG, "track boundary: skipped %ld ms to null phase",
                             (long)(applied * 1000 / (int32_t)stream_rate));
                    phase_stepped = true;
                } else if (adj < 0) {
                    /* Early: emit silence so the timeline catches up with us. */
                    static const uint8_t quiet[AUDIO_CHUNK_BYTES] = {0};
                    int32_t left = -adj;
                    while (left > 0) {
                        int32_t n = left > (int32_t)AUDIO_FRAMES ? (int32_t)AUDIO_FRAMES : left;
                        write_audio(quiet, (size_t)n * AUDIO_CHANNELS * sizeof(int16_t));
                        left -= n;
                    }
                    applied = adj;
                    ESP_LOGW(TAG, "track boundary: inserted %ld ms to null phase",
                             (long)(-applied * 1000 / (int32_t)stream_rate));
                    phase_stepped = true;
                }

                /*
                 * Hand the correction to the probe task to report. Not sent
                 * from here: a sendto() in the playback path is exactly the
                 * kind of thing that costs a buffer, and this is not urgent --
                 * the hub only wants it to print one line per track.
                 */
                if (applied != 0) {
                    n_splices++;
                    /* Same reason phase_stepped is set above: every reading in
                     * the history was taken before this unit moved. */
                    sync_phase_reset(&phase_hist);
                }
                splice_report_us = (int32_t)((int64_t)applied * 1000000 / stream_rate);
                splice_report_phase = phase_valid ? phase_err_us : 0;
                splice_report_pending = true;
                /*
                 * The visualiser is not told either. A splice moves audio around
                 * WITHIN the timeline to correct this unit's position in it; the
                 * timeline itself, which is what every frame is dated against and
                 * drawn on, does not move. Arrival is untouched.
                 */
            }

            int32_t mark = marker_sample;
            if (mark >= 0 && samples_played >= mark) {
#if CONFIG_DANCEFLOOR_ENABLE_MARKER
                /* 200 us of busy-wait in the playback path. Worth it while a
                 * hub is wired to the other end and nothing else can measure
                 * what reaches the speaker; pure cost once the wire is off. */
                gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 1);
                esp_rom_delay_us(MARKER_PULSE_US);
                gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 0);
#endif
                marker_sample = -1;
            }
            samples_played += AUDIO_FRAMES;

            /* Last thing before the output, and deliberately after every count
             * above: it rewrites slots within frames that already exist, so
             * samples_played and the phase queue are looking at the same
             * timeline whatever this unit's speaker is placed as.
             *
             * Here rather than inside write_audio() because that is also called
             * with the const `quiet` buffer a splice inserts, which must not be
             * written to -- and needs nothing doing to it, since every mode
             * maps silence to silence. */
            audio_apply_channel_mode((int16_t *)chunk, AUDIO_FRAMES);

            write_audio(chunk, sizeof(chunk));
            /* Immediately: it is the instant the next pass dates its phase
             * reading from. */
            wrote_at = esp_timer_get_time();
        }
    }
}
