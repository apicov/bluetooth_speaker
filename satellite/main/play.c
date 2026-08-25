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
#include "audio_shift.h"
#include "sbc_link.h"
#include "sbc_decoder.h"
#include "visualiser.h"
#include "wifi_log.h"

#include "sat.h"

#define TRIM_ONE_FRAME 65536
static int32_t s_trim_owed;

static int32_t s_phase_prev;
static bool    s_phase_prev_valid;

#define PHASE_STEP_LOG_US 20000

static int trim_due(void)
{
    const int32_t hz = rate_trim_hz;
    if (hz == 0) {

        s_trim_owed = 0;
        return 0;
    }
    s_trim_owed += (int32_t)((int64_t)AUDIO_FRAMES * hz * TRIM_ONE_FRAME
                             / (int64_t)tx_rate);

    if (s_trim_owed >  2 * TRIM_ONE_FRAME) s_trim_owed =  2 * TRIM_ONE_FRAME;
    if (s_trim_owed < -2 * TRIM_ONE_FRAME) s_trim_owed = -2 * TRIM_ONE_FRAME;
    if (s_trim_owed >= TRIM_ONE_FRAME) {
        s_trim_owed -= TRIM_ONE_FRAME;
        return 1;
    }
    if (s_trim_owed <= -TRIM_ONE_FRAME) {
        s_trim_owed += TRIM_ONE_FRAME;
        return -1;
    }
    return 0;
}

static int16_t s_cu_in[(AUDIO_FRAMES + CATCHUP_SHIFT_MAX + 1) * AUDIO_CHANNELS];

static int s_catchup_take;

static int32_t s_catchup_moved;

static int chunk_shift(void)
{
    s_catchup_take = 0;
    const int trim = trim_due();

    const int32_t debt = catchup_frames;
    if (debt == 0) {
        return trim;
    }

    const int32_t level_ms = (int32_t)((int64_t)(RING_BYTES -
            (int64_t)xStreamBufferSpacesAvailable(ring)) * 1000
            / ((int64_t)stream_rate * AUDIO_CHANNELS * 2));
    if (debt > 0 && level_ms <= RING_TARGET_MS - 30) {
        return trim;
    }
    if (debt < 0 && level_ms >= RING_TARGET_MS + 50) {
        return trim;
    }

    int32_t k = debt;
    if (k >  CATCHUP_SHIFT_MAX_DROP) k =  CATCHUP_SHIFT_MAX_DROP;
    if (k < -CATCHUP_SHIFT_MAX_DUP)  k = -CATCHUP_SHIFT_MAX_DUP;
    s_catchup_take = (int)k;
    return (int)k + trim;
}

static uint8_t vol_now(void)
{
    static bool told;
    const bool due = esp_timer_get_time() >= AUDIO_VOL_UNKNOWN_HOLD_US;
    if (due && !audio_vol_known && !told) {
        told = true;
        ESP_LOGE(TAG, "NO VOLUME in %d s -- nothing has told this unit a level. "
                      "Falling back to FULL SCALE.",
                 (int)(AUDIO_VOL_UNKNOWN_HOLD_US / 1000000));
    }
    return audio_vol_effective(audio_volume, audio_vol_known, due);
}

void play_task(void *arg)
{
    (void)arg;
    static uint8_t chunk[AUDIO_CHUNK_BYTES];

    while (1) {
        if (stream_start_local == 0) {

            resync_request = false;
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        int64_t wait = stream_start_local - esp_timer_get_time();
        if (wait > 2000) {
            vTaskDelay(pdMS_TO_TICKS((wait - 2000) / 1000));
        }
        while (esp_timer_get_time() < stream_start_local) {
        }
        int64_t actual_master = esp_timer_get_time() + stream_offset;
        int64_t sched_master = stream_start_local + stream_offset;
        ESP_LOGI(TAG, "playback started: scheduled %lld, actual %lld (%+lld us) [master]",
                 sched_master, actual_master, actual_master - sched_master);

        int32_t samples_played = 0;

        s_trim_owed = 0;

        sync_phase_reset(&phase_hist);

        catchup_frames = 0;

        s_phase_prev_valid = false;

        s_refill_active = true;
        s_refill_frames = 0;

        int64_t wrote_at = esp_timer_get_time();

        write_audio_reset_ramp();

        playing = true;

        while (1) {
            if (retuning) {

                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }

            if (resync_request) {
                resync_request = false;
                stream_start_local = 0;
                phase_valid = false;
                phase_stepped = true;
                break;
            }

            const int shift = chunk_shift();
            const bool cu = (shift > 1 || shift < -1);
            const size_t frame_bytes = AUDIO_CHANNELS * sizeof(int16_t);
            const size_t want = cu
                ? (size_t)((int)AUDIO_FRAMES + shift) * frame_bytes
                : (shift < 0) ? sizeof(chunk) - frame_bytes : sizeof(chunk);

            hw_play = uxTaskGetStackHighWaterMark(NULL);

            {
                const int32_t level_ms = (int32_t)((int64_t)(RING_BYTES -
                        (int64_t)xStreamBufferSpacesAvailable(ring)) * 1000
                        / ((int64_t)stream_rate * AUDIO_CHANNELS * 2));
                if (level_ms < ring_low_ms) {
                    ring_low_ms = level_ms;
                }
            }
            uint8_t *const dest = cu ? (uint8_t *)s_cu_in : chunk;
            size_t got = xStreamBufferReceive(ring, dest, want, pdMS_TO_TICKS(500));
            if (got == 0) {
                n_underruns++;
                ESP_LOGW(TAG, "underrun, waiting for a new stream");
                stream_start_local = 0;
                break;
            }

            int32_t consumed = (int32_t)(got / frame_bytes);
            if (got < want) {
                n_short_reads++;
                n_short_frames += (uint32_t)((want - got) / frame_bytes);
            }

            size_t have = got;
            if (cu && got == want) {

                audio_shift_chunk((int16_t *)chunk, s_cu_in, AUDIO_FRAMES,
                                  shift, CATCHUP_FADE_FRAMES, AUDIO_CHANNELS);
                consumed = (int32_t)AUDIO_FRAMES + shift;
                have = sizeof(chunk);
                catchup_frames -= s_catchup_take;
                if (shift > 0) n_catchup_drops += (uint32_t)shift;
                else           n_catchup_dups  += (uint32_t)(-shift);

                s_catchup_moved += s_catchup_take < 0 ? -s_catchup_take
                                                      : s_catchup_take;
                if (s_catchup_moved >= (int32_t)((int64_t)CATCHUP_HIST_RESET_US *
                                                 stream_rate / 1000000)) {
                    s_catchup_moved = 0;
                    sync_phase_reset(&phase_hist);
                    phase_stepped = true;
                }
            } else if (cu) {

                const size_t copy = got < sizeof(chunk) ? got : sizeof(chunk);
                memcpy(chunk, s_cu_in, copy);
                have = copy;
            }
            if (have < sizeof(chunk)) {
                memset(chunk + have, 0, sizeof(chunk) - have);
            }

            if (!cu && shift > 0) {

                static uint8_t dropped[AUDIO_CHANNELS * sizeof(int16_t)];
                if (xStreamBufferReceive(ring, dropped, sizeof(dropped), 0)
                    == sizeof(dropped)) {
                    consumed++;
                    n_trim_drops++;
                } else {
                    s_trim_owed += TRIM_ONE_FRAME;
                }
            } else if (!cu && shift < 0) {

                memcpy(chunk + want, chunk + want - frame_bytes, frame_bytes);
                n_trim_dups++;
            }

            track_offset();

            bool timeline_changed = false;
            while (phase_tail != phase_head && samples_played >= phase_q[phase_tail].pos) {
                int64_t due = phase_q[phase_tail].play_at;

                int32_t overshoot = samples_played - phase_q[phase_tail].pos;

                if (overshoot > (int32_t)AUDIO_FRAMES) {
                    overshoot = (int32_t)AUDIO_FRAMES;
                }

                int64_t crossed_at = wrote_at
                                   - (int64_t)overshoot * 1000000 / stream_rate;
                int64_t now_master = crossed_at + stream_offset;
                int64_t err = now_master - due;

                if (err > PHASE_INSANE_US || err < -PHASE_INSANE_US) {
                    ESP_LOGE(TAG, "phase %lld us -- not the timeline we anchored to, "
                                  "re-anchoring", err);
                    timeline_changed = true;
                    break;
                }

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

                    if (retune_tail_left) {
                        retune_tail_left--;
                        ESP_LOGW(TAG, "RETUNE TAIL: phase %+lld us at %lld us after "
                                      "the retune (net %+lld from before it)",
                                 err, since_retune, err - retune_phase_before);
                    }

                    if (s_phase_prev_valid) {
                        const int32_t step = (int32_t)err - s_phase_prev;
                        const int32_t mag = step < 0 ? -step : step;
                        if (mag > PHASE_STEP_LOG_US && mag > step_report_mag) {
                            step_report_mag  = mag;
                            step_report_from = s_phase_prev;
                            step_report_to   = (int32_t)err;
                            step_report_ring = (int32_t)((RING_BYTES -
                                xStreamBufferSpacesAvailable(ring)) * 1000
                                / (stream_rate * AUDIO_CHANNELS * 2));
                            step_report_pad  = n_short_frames;
                            step_report_trim = rate_trim_hz;
                            step_report_pending = true;
                        }
                    }
                    s_phase_prev = (int32_t)err;
                    s_phase_prev_valid = true;

                    phase_err_us = (int32_t)err;
                    phase_valid = true;
                    sync_phase_push(&phase_hist, (int32_t)err);
                }
                phase_tail = (phase_tail + 1) % PHASE_Q_LEN;
            }
            if (timeline_changed) {

                stream_start_local = 0;
                phase_valid = false;
                phase_stepped = true;
                break;
            }

            int32_t rp = restart_pos;
            if (rp >= 0 && samples_played >= rp) {
                restart_pos = -1;
                int32_t max_frames = (int32_t)stream_rate * MAX_SPLICE_MS / 1000;

                int32_t med_us = 0;
                const bool have_med = phase_valid &&
                                      sync_phase_median(&phase_hist, &med_us);
                const int32_t splice_us = have_med ? med_us : (int32_t)phase_err_us;

                int32_t adj = phase_valid
                    ? (int32_t)((int64_t)splice_us * stream_rate / 1000000) : 0;
                if (adj > max_frames)  adj = max_frames;
                if (adj < -max_frames) adj = -max_frames;
                int32_t applied = 0;

                int32_t raw_adj = phase_valid
                    ? (int32_t)((int64_t)phase_err_us * stream_rate / 1000000) : 0;
                if (raw_adj > max_frames)  raw_adj = max_frames;
                if (raw_adj < -max_frames) raw_adj = -max_frames;
                splice_report_alt = (int32_t)((int64_t)raw_adj * 1000000 / stream_rate);

                if (adj > 0) {

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

                    const int32_t level_frames = (int32_t)((RING_BYTES -
                            xStreamBufferSpacesAvailable(ring))
                            / (AUDIO_CHANNELS * (int)sizeof(int16_t)));
                    const int32_t room = (int32_t)(RING_BYTES /
                            (AUDIO_CHANNELS * (int)sizeof(int16_t)))
                            - (int32_t)stream_rate * SPLICE_INSERT_HEADROOM_MS / 1000
                            - level_frames;
                    if (-adj > room) {
                        ESP_LOGW(TAG, "insert clamped: phase asked %ld ms, ring "
                                      "has room for %ld ms",
                                 (long)(-adj * 1000 / (int32_t)stream_rate),
                                 (long)((room > 0 ? room : 0) * 1000
                                        / (int32_t)stream_rate));
                        adj = room > 0 ? -room : 0;
                    }

                    static const int16_t quiet[AUDIO_FRAMES * AUDIO_CHANNELS] = {0};
                    _Static_assert(sizeof(quiet) == AUDIO_CHUNK_BYTES,
                                   "the splice's silence is one ring-domain chunk");
                    int32_t left = -adj;
                    while (left > 0) {
                        int32_t n = left > (int32_t)AUDIO_FRAMES ? (int32_t)AUDIO_FRAMES : left;

                        write_audio(quiet, (size_t)n, AUDIO_VOL_MAX);
                        left -= n;
                    }
                    applied = adj;
                    ESP_LOGW(TAG, "track boundary: inserted %ld ms to null phase",
                             (long)(-applied * 1000 / (int32_t)stream_rate));
                    phase_stepped = true;
                }

                if (applied != 0) {
                    n_splices++;

                    sync_phase_reset(&phase_hist);

                    catchup_frames = 0;
                }
                splice_report_us = (int32_t)((int64_t)applied * 1000000 / stream_rate);
                splice_report_phase = phase_valid ? phase_err_us : 0;
                splice_report_pending = true;

            }

            int32_t mark = marker_sample;
            if (mark >= 0 && samples_played >= mark) {
#if CONFIG_DANCEFLOOR_ENABLE_MARKER

                gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 1);
                esp_rom_delay_us(MARKER_PULSE_US);
                gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 0);
#endif
                marker_sample = -1;
            }

            samples_played += consumed;

            audio_apply_channel_mode((int16_t *)chunk, AUDIO_FRAMES);

            write_audio((const int16_t *)chunk, AUDIO_FRAMES, vol_now());

            wrote_at = esp_timer_get_time();
        }

        playing = false;
    }
}
