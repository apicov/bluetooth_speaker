/**
 * @file telemetry.c
 * @brief One 5 s window of reporting, and the allocator failure hook.
 *
 * The audio path counts and this file talks. Every counter here is incremented
 * somewhere that must not touch a UART: rx_task is the only thing draining the
 * UDP mailbox, and play_task is the audio path, so a log line from either
 * costs buffer. This task can afford to wait on a 115200-baud console.
 *
 * @section format The lines are a wire format, not decoration
 *
 * tools/soak/capture.py classifies a line by its literal prefix — `HEALTH:`,
 * `TRIM:`, `MEM:`, `RX 5s:`, `ARRIVAL 5s:` — and then turns it into metrics
 * columns with one regex over `key then number` pairs. So a key must be a
 * single hyphenated word immediately followed by its value, and renaming or
 * reordering one silently drops a column from every soak. Tidying these
 * strings breaks a tool.
 *
 * @section cadence What prints when
 *
 * - `ARRIVAL 5s:` every window, unconditionally. It is the line that says the
 *   unit is alive and how audio is reaching it, and a fault that stops audio
 *   arriving would also stop a conditional line from printing.
 * - `RX 5s:` only when a fault counter moved, differenced against what it last
 *   said.
 * - `HEALTH:`, `TRIM:` and `MEM:` once a minute.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "sbc_link.h"
#include "sync_proto.h"
#include "visualiser.h"
#include "wifi_log.h"

#include "sat.h"

/* on_alloc_failed() is documented at its declaration in sat.h.
 *
 * Records and does not log, for two reasons. It runs INSIDE the allocator, and
 * ESP_LOGx allocates. And IDF marks the heap's failure path IRAM-resident
 * because the heap stays usable with the flash cache disabled -- so a hook in
 * flash would fault when reached from an ISR or during a flash write, which is
 * a diagnostic for running out of memory that crashes under the one condition
 * it exists to observe.
 */
IRAM_ATTR void on_alloc_failed(size_t size, uint32_t caps, const char *function_name)
{
    (void)function_name;
    n_alloc_fail++;
    if ((uint32_t)size > alloc_fail_size) {
        alloc_fail_size = (uint32_t)size;
        alloc_fail_caps = caps;
    }
}

/* telemetry_tick() is documented at its declaration in sat.h. */
void telemetry_tick(void)
{
    const uint32_t heap_now = esp_get_free_heap_size();
    if (heap_now < heap_min_window) {
        heap_min_window = heap_now;
    }
    const uint32_t heap_int_now = (uint32_t)heap_caps_get_free_size(CAP_USABLE_INTERNAL);
    if (heap_int_now < heap_int_window) {
        heap_int_window = heap_int_now;
    }

    /* Said as soon as it happens rather than waiting for the minute line: an
     * allocation failure is the kind of fault whose next consequence is a task
     * that does not start. */
    static uint32_t alloc_fail_told;
    if (n_alloc_fail != alloc_fail_told) {
        alloc_fail_told = n_alloc_fail;
        ESP_LOGE(TAG, "ALLOCATION FAILED %" PRIu32 " time(s): largest request %"
                      PRIu32 " B (caps 0x%" PRIx32 "%s%s%s) | internal %u free "
                      "(min %u), largest %u | total %" PRIu32 " free",
                 n_alloc_fail, alloc_fail_size, alloc_fail_caps,
                 (alloc_fail_caps & MALLOC_CAP_INTERNAL) ? " INTERNAL" : "",
                 (alloc_fail_caps & MALLOC_CAP_DMA)      ? " DMA"      : "",
                 (alloc_fail_caps & MALLOC_CAP_SPIRAM)   ? " SPIRAM"   : "",
                 (unsigned)heap_int_now,
                 (unsigned)heap_caps_get_minimum_free_size(CAP_USABLE_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(CAP_USABLE_INTERNAL),
                 heap_now);
    }

    if (step_report_pending) {
        step_report_pending = false;
        static uint32_t step_pad_told;
        /* Read once: the play task is still writing it. Printed as a delta
         * against the last step, because the running total accumulates across
         * hours and says nothing about the step being reported. */
        const uint32_t pad_total = step_report_pad;
        ESP_LOGW(TAG, "PHASE STEP: %+ld -> %+ld us (%+ld) | buffer %ld ms | "
                      "pad %" PRIu32 " frames | trim %+ld Hz",
                 (long)step_report_from, (long)step_report_to,
                 (long)(step_report_to - step_report_from),
                 (long)step_report_ring,
                 pad_total - step_pad_told,
                 (long)step_report_trim);
        step_pad_told = pad_total;
        step_report_mag = 0;
    }

    /* Snapshot every counter, print the window's differences only if one
     * moved, then update all of them regardless -- so a window that printed
     * nothing does not fold its silence into the next one's numbers. */
    static uint32_t gaps_told, gap_frames_told, gap_short_told,
                    gap_short_frames_told, ring_full_told,
                    anchor_late_told, anchor_soon_told, gap_resyncs_told,
                    upgrades_told, fec_told, fec_lost_told, fec_holds_told,
                    fec_bad_told, short_frames_told;
    const uint32_t short_frames_now = n_short_frames;
    const uint32_t gaps_now = n_gaps, gap_frames_now = n_gap_frames,
                   gap_short_now = n_gap_short,
                   gap_short_frames_now = n_gap_short_frames,
                   ring_full_now = n_ring_full,
                   anchor_late_now = n_anchor_late,
                   anchor_soon_now = n_anchor_soon,
                   gap_resyncs_now = n_gap_resyncs,
                   upgrades_now = n_anchor_upgrades,
                   fec_now = n_fec_recovered,
                   fec_lost_now = n_fec_lost,
                   fec_holds_now = n_fec_holds,
                   fec_bad_now = n_fec_bad;
    if (gaps_now != gaps_told || ring_full_now != ring_full_told ||
        anchor_late_now != anchor_late_told || anchor_soon_now != anchor_soon_told ||
        gap_resyncs_now != gap_resyncs_told || upgrades_now != upgrades_told ||
        fec_now != fec_told || fec_lost_now != fec_lost_told ||
        fec_holds_now != fec_holds_told || fec_bad_now != fec_bad_told ||
        short_frames_now != short_frames_told) {
        ESP_LOGW(TAG, "RX 5s: gaps %" PRIu32 " (%" PRIu32 " ms silence, %"
                      PRIu32 " short by %" PRIu32 " ms) | ring-full %" PRIu32
                      " | pad %" PRIu32 " ms"
                      " | too big to fill %" PRIu32 " | upgrades %" PRIu32
                      " | anchors refused %" PRIu32 " late, %" PRIu32 " too soon"
                      " | fec %" PRIu32 " (fec-lost %" PRIu32 ", fec-held %"
                      PRIu32 ", fec-bad %" PRIu32 ")",
                 gaps_now - gaps_told,
                 (gap_frames_now - gap_frames_told) * 1000 / stream_rate,
                 gap_short_now - gap_short_told,
                 (gap_short_frames_now - gap_short_frames_told) * 1000 / stream_rate,
                 ring_full_now - ring_full_told,
                 (short_frames_now - short_frames_told) * 1000 / stream_rate,
                 gap_resyncs_now - gap_resyncs_told,
                 upgrades_now - upgrades_told,
                 anchor_late_now - anchor_late_told,
                 anchor_soon_now - anchor_soon_told,
                 fec_now - fec_told,
                 fec_lost_now - fec_lost_told,
                 fec_holds_now - fec_holds_told,
                 fec_bad_now - fec_bad_told);
    }
    gaps_told = gaps_now;
    gap_frames_told = gap_frames_now;
    gap_short_told = gap_short_now;
    gap_short_frames_told = gap_short_frames_now;
    ring_full_told = ring_full_now;
    anchor_late_told = anchor_late_now;
    anchor_soon_told = anchor_soon_now;
    gap_resyncs_told = gap_resyncs_now;
    upgrades_told = upgrades_now;
    fec_told = fec_now;
    fec_lost_told = fec_lost_now;
    fec_holds_told = fec_holds_now;
    fec_bad_told = fec_bad_now;
    short_frames_told = short_frames_now;

    static uint32_t audio_rx_told, starve_told, lead_insane_told;
    const uint32_t audio_rx_now = n_audio_rx, starve_now = dma_starve_count();
    const uint32_t lead_insane_now = n_lead_insane;
    const int32_t gap_max = rx_gap_max_us;
    const uint32_t burst_max = rx_burst_max;
    const int32_t lead_min = rx_lead_min_us;
    const int32_t ring_low = ring_low_ms;
    const int32_t hold_max = fec_hold_max_us;
    /* Cleared as they are read, so each line is its own window: a maximum
     * summed across windows is not a maximum of anything. The read and the
     * clear are not atomic against the tasks that write them, and losing one
     * sample of an extreme to that race costs a diagnostic nothing -- which is
     * why these are gauges and the counters above are not. */
    rx_gap_max_us = 0;
    rx_burst_max = 0;
    rx_lead_min_us = ARRIVAL_UNSEEN;
    ring_low_ms = ARRIVAL_UNSEEN;
    fec_hold_max_us = 0;

    /* Each starve callback is one DMA descriptor's worth of digital zero, and
     * i2s_start() sets the descriptor to AUDIO_FRAMES -- so the count means
     * nothing until it is turned into time. */
    const uint32_t starved_ms = (starve_now - starve_told) * AUDIO_FRAMES
                              * 1000 / stream_rate;

    /* Printed as text so that "nothing measured this window" and "measured
     * zero" cannot be read as each other. Zero lead is exactly the reading
     * that matters, so it must not share a spelling with no reading at all. */
    char lead_s[16], ring_s[16];
    if (lead_min == ARRIVAL_UNSEEN) {
        snprintf(lead_s, sizeof(lead_s), "none");
    } else {
        snprintf(lead_s, sizeof(lead_s), "%+ld ms", (long)(lead_min / 1000));
    }
    if (ring_low == ARRIVAL_UNSEEN) {
        snprintf(ring_s, sizeof(ring_s), "none");
    } else {
        snprintf(ring_s, sizeof(ring_s), "%ld ms", (long)ring_low);
    }

    /* The DOWNLINK signal: how well this unit hears the hub. The hub's own
     * report measures the uplink, and the two should read roughly symmetric
     * because antenna gain is reciprocal -- a pair that does not is worth
     * seeing. It is also the number the self-mute decides on. */
    wifi_ap_record_t ap;
    char rssi_s[16];
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        snprintf(rssi_s, sizeof(rssi_s), "%d dBm", (int)ap.rssi);
    } else {
        snprintf(rssi_s, sizeof(rssi_s), "none");
    }

    /* fec-parity rides this line and not RX 5s deliberately. On that line it
     * would be invisible in exactly the windows that matter, because RX prints
     * only when a fault counter moved -- and "the hub stopped sending parity"
     * moves no fault counter on this unit at all. */
    static uint32_t fec_parity_told;
    const uint32_t fec_parity_now = n_fec_parity_rx;

    ESP_LOGW(TAG, "ARRIVAL 5s: pkts %" PRIu32 " | fec-parity %" PRIu32
                  " | gap-max %ld ms | "
                  "burst-max %" PRIu32 " | lead-min %s | lead-drop %" PRIu32
                  " | ring-low %s | fec-hold-max %ld ms"
                  " | starved %" PRIu32 " ms | hub-rssi %s%s",
             audio_rx_now - audio_rx_told, fec_parity_now - fec_parity_told,
             (long)(gap_max / 1000), burst_max, lead_s,
             lead_insane_now - lead_insane_told, ring_s,
             (long)(hold_max / 1000), starved_ms, rssi_s,

             /* On the every-window line, because a unit that has taken itself
              * off the floor looks exactly like a dead one from anywhere
              * else. */
             self_muted ? "  ** SELF-MUTED, off the hub's send list **" : "");
    audio_rx_told = audio_rx_now;
    fec_parity_told = fec_parity_now;
    starve_told = starve_now;
    lead_insane_told = lead_insane_now;

    /* Once a minute: twelve of these 5 s windows. */
    static int health_left;
    if (--health_left <= 0) {
        health_left = 12;
        /* Only valid in-task, which is why it is sampled here and not
         * wherever it is printed. */
        hw_drift = uxTaskGetStackHighWaterMark(NULL);

        /* Taken and reset together with the all-time minimum beside it: the
         * pair dates a dip to this line, which a watermark alone never
         * could. */
        const uint32_t heap_win = heap_min_window;
        heap_min_window = UINT32_MAX;
        const uint32_t heap_int_win = heap_int_window;
        heap_int_window = UINT32_MAX;
        ESP_LOGW(TAG, "HEALTH: up %llu s | heap %" PRIu32 " (min %" PRIu32
                      ", window %" PRIu32 ", largest %u) | "
                      "stack play %" PRIu32 " drift %" PRIu32 " | underruns %" PRIu32
                      " anchors %" PRIu32 " splices %" PRIu32 " retunes %" PRIu32
                      " (%" PRIu32 " refused) | gaps %" PRIu32 " (%" PRIu32
                      " short, %" PRIu32 " too big) ring-full %" PRIu32 " upgrades %" PRIu32 " anchors-refused %" PRIu32
                      " | wifi-drops %" PRIu32 " (no-lease %" PRIu32 ")"
                      " | alloc-fail %" PRIu32
                      " | clock %s (tsf %" PRIu32 "/probe %" PRIu32
                      ", wide-span %" PRIu32 ", tsf-read-fail %" PRIu32 ")"
                      " | self-mutes %" PRIu32 " (retries %" PRIu32 ")"
                      " | phase-drop %" PRIu32 " short-reads %" PRIu32
                      " (%" PRIu32 " frames)"
                      " | dma-starve %" PRIu32 " short-resync %" PRIu32
                      " | seq-drop %" PRIu32 " decode-err %" PRIu32
                      " recv-err %" PRIu32
                 " | leds %s hop %d (rx %" PRIu32 ", bad %" PRIu32 ")",
                 (unsigned long long)(esp_timer_get_time() / 1000000),
                 esp_get_free_heap_size(), esp_get_minimum_free_heap_size(),
                 heap_win == UINT32_MAX ? 0 : heap_win,
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT),
                 hw_play, hw_drift, n_underruns, n_reanchors, n_splices,
                 n_retunes, n_retunes_bad, n_gaps, n_gap_short, n_gap_resyncs, n_ring_full,
                 n_anchor_upgrades,
                 n_anchor_late + n_anchor_soon, n_wifi_drops, n_wifi_lease_fail,
                 n_alloc_fail,
                 tsf_fresh(esp_timer_get_time(), NULL) ? "TSF" : "probe",
                 n_tsf_used, n_tsf_fallback, n_tsf_wide, n_tsf_read_fail,
                 n_self_mutes, n_self_retries,
                 n_phase_drop, n_short_reads, n_short_frames,
                 dma_starve_count(), n_gap_short_resyncs,
                 n_seq_dropped, n_decode_err, n_recv_err,
                 visualiser_source_name(), visualiser_hop(),
                 n_frames_rx, n_frames_bad);

        /* The catch-up gets its own keys rather than riding the trim totals:
         * a drain deliberately exceeds any rate the trim could claim, so
         * summing them would hide the trim entirely. */
        ESP_LOGW(TAG, "TRIM: %+ld Hz | dropped %" PRIu32 " dup %" PRIu32
                      " frames | catchup-drops %" PRIu32 " catchup-dups %"
                      PRIu32 " | retunes %" PRIu32 " coarse | volume %u/%d "
                      "vol-rx %" PRIu32,
                 (long)rate_trim_hz, n_trim_drops, n_trim_dups,
                 n_catchup_drops, n_catchup_dups, n_retunes,
                 audio_volume, AUDIO_VOL_MAX, n_vol_rx);

        ESP_LOGW(TAG, "MEM: internal %u free (min %u, window %" PRIu32
                      ", largest %u) | total %" PRIu32 " (largest %u)",
                 (unsigned)heap_caps_get_free_size(CAP_USABLE_INTERNAL),
                 (unsigned)heap_caps_get_minimum_free_size(CAP_USABLE_INTERNAL),
                 heap_int_win == UINT32_MAX ? 0 : heap_int_win,
                 (unsigned)heap_caps_get_largest_free_block(CAP_USABLE_INTERNAL),
                 esp_get_free_heap_size(),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));

#if CONFIG_DANCEFLOOR_WIFI_LOGS
        /* The same figures as the HEALTH line above, structured, for the
         * collector. Some field names are shared with the hub's health message
         * and mean different things on each -- the struct is one wire format
         * for two roles. */
        static uint32_t health_seq;
        health_msg_t h;
        memset(&h, 0, sizeof h);
        h.type = MSG_HEALTH;
        h.role = LOG_ROLE_SAT;
        h.clock_src = tsf_fresh(esp_timer_get_time(), NULL) ? 1 : 0;
        h.seq = health_seq++;
        h.uptime_s = (uint64_t)(esp_timer_get_time() / 1000000);
        h.heap_cur = esp_get_free_heap_size();
        h.heap_min = esp_get_minimum_free_heap_size();
        h.heap_win = heap_win == UINT32_MAX ? 0 : heap_win;
        h.heap_largest = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
        h.hw_play = hw_play;
        h.hw_mon = hw_drift;
        h.underruns = n_underruns;
        h.reanchors_or_restarts = n_reanchors;
        h.splices = n_splices;
        h.retunes = n_retunes;
        h.retunes_refused = n_retunes_bad;
        h.gaps_or_sta_left = n_gaps;
        h.wifi_drops_or_oversize = n_wifi_drops;
        h.alloc_fail = n_alloc_fail;
        h.phase_drop = n_phase_drop;
        h.short_reads = n_short_reads;
        h.short_frames = n_short_frames;
        h.ring_full_or_sta_dropped = n_ring_full;
        h.upgrades_or_sta_nolease = n_anchor_upgrades;
        h.anchors_refused_or_timeout = n_anchor_late + n_anchor_soon;
        h.log_dropped = wifi_log_dropped();
        h.log_no_dest = wifi_log_no_dest();
        wifi_log_send_to_dest(&h, sizeof h);
#endif
    }
}
