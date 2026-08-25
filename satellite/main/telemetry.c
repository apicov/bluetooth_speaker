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

IRAM_ATTR void on_alloc_failed(size_t size, uint32_t caps, const char *function_name)
{
    (void)function_name;
    n_alloc_fail++;
    if ((uint32_t)size > alloc_fail_size) {
        alloc_fail_size = (uint32_t)size;
        alloc_fail_caps = caps;
    }
}

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
    rx_gap_max_us = 0;
    rx_burst_max = 0;
    rx_lead_min_us = ARRIVAL_UNSEEN;
    ring_low_ms = ARRIVAL_UNSEEN;
    fec_hold_max_us = 0;

    const uint32_t starved_ms = (starve_now - starve_told) * AUDIO_FRAMES
                              * 1000 / stream_rate;

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

    wifi_ap_record_t ap;
    char rssi_s[16];
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        snprintf(rssi_s, sizeof(rssi_s), "%d dBm", (int)ap.rssi);
    } else {
        snprintf(rssi_s, sizeof(rssi_s), "none");
    }

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

             self_muted ? "  ** SELF-MUTED, off the hub's send list **" : "");
    audio_rx_told = audio_rx_now;
    fec_parity_told = fec_parity_now;
    starve_told = starve_now;
    lead_insane_told = lead_insane_now;

    static int health_left;
    if (--health_left <= 0) {
        health_left = 12;
        hw_drift = uxTaskGetStackHighWaterMark(NULL);

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
