/*
 * Everything this unit says about itself: the windowed heap figures, the
 * allocation-failure report, the receive path's 5 s window, and the HEALTH and
 * MEM lines a long run is judged on.
 *
 * It lives apart from the servo it used to be nested inside. Why it runs on
 * that task is unchanged and still matters -- narrating from the receive path
 * is what closes the feedback loop the RX counters exist to undo, so the
 * talking has to happen somewhere that can afford to block on a UART -- but
 * "a task that can afford to wait" is a scheduling property, not a reason for
 * rate control and health reporting to share a function body.
 *
 * Called once per 5 s window by drift_task, before the servo, in the order the
 * single-file version ran them. Split out of main.c on 2026-08-12; the body is
 * unchanged apart from one level of indentation.
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

/*
 * Records only, and in IRAM. IDF marks heap_caps_alloc_failed() HEAP_IRAM_ATTR
 * because the heap is usable with the flash cache disabled, so a hook in flash
 * would fault when reached from an ISR or during a flash write -- a diagnostic
 * for running out of memory that crashes under the one condition it exists to
 * observe. It also runs inside the allocator, and ESP_LOGx allocates, so
 * drift_task does the talking within 5 s.
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

    /* Said once, and within 5 s of the fact rather than at the next soak
     * line -- see the hub's copy, which also carries why this reports both
     * pools, why `min` is the figure that explains a failure when the live
     * ones no longer can, and why the hook cannot sample them itself. */
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

    /*
     * The largest phase step playback saw this window.
     *
     * Recorded there, printed here, for the reason the RX counters below are:
     * the play task is the audio path. Cleared as it is taken, so a quiet
     * window says nothing at all.
     *
     * pad is the WINDOW delta, differenced against the last value told. The
     * play task records the since-boot cumulative n_short_frames, and printing
     * that raw made every PHASE STEP line claim all padding since power-on --
     * on a long soak the first step of the evening read 25299 frames of pad
     * accumulated by delivery bursts hours old, which is a HEALTH figure, not
     * a step figure. The delta here is the padding that rode along with THIS
     * step; the cumulative keeps its home on the HEALTH line.
     */
    if (step_report_pending) {
        step_report_pending = false;
        static uint32_t step_pad_told;
        const uint32_t pad_total = step_report_pad;   /* read once: it moves */
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

    /*
     * The receive path's window, said here because it cannot afford to say
     * it itself -- see the counters' declaration.
     *
     * Only when something moved, so a healthy run stays quiet, and one line
     * per 5 s window however bad it gets. That bound is the point: the
     * failure this replaces produced output in proportion to the damage,
     * from the task that had to stop the damage.
     */
    /*
     * short_frames rides on this line as well as the 60 s HEALTH totals.
     *
     * It belongs beside gaps because it is the other half of the same event and
     * the only one that moves this unit PERMANENTLY: a short read pads the DAC
     * to a full chunk, the pad takes DAC time, and samples_played does not count
     * it -- so every padded frame is a frame of the timeline this unit will
     * never get back. The soak that found the delivery-burst fault accumulated
     * 25299 of them, 574 ms, and it was only visible by differencing two HEALTH
     * lines a minute apart. As a per-window delta beside the gaps that caused
     * it, it reads directly.
     */
    static uint32_t gaps_told, gap_frames_told, gap_short_told,
                    gap_short_frames_told, ring_full_told,
                    anchor_late_told, anchor_soon_told, gap_resyncs_told,
                    upgrades_told, fec_told, fec_short_told, fec_err_told,
                    short_frames_told;
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
                   fec_short_now = n_fec_short_frames,
                   fec_err_now = n_fec_decode_err;
    if (gaps_now != gaps_told || ring_full_now != ring_full_told ||
        anchor_late_now != anchor_late_told || anchor_soon_now != anchor_soon_told ||
        gap_resyncs_now != gap_resyncs_told || upgrades_now != upgrades_told ||
        fec_now != fec_told || fec_short_now != fec_short_told ||
        fec_err_now != fec_err_told || short_frames_now != short_frames_told) {
        ESP_LOGW(TAG, "RX 5s: gaps %" PRIu32 " (%" PRIu32 " ms silence, %"
                      PRIu32 " short by %" PRIu32 " ms) | ring-full %" PRIu32
                      " | pad %" PRIu32 " ms"
                      " | too big to fill %" PRIu32 " | upgrades %" PRIu32
                      " | anchors refused %" PRIu32 " late, %" PRIu32 " too soon"
                      " | fec %" PRIu32 " (%" PRIu32 " ms short, %" PRIu32 " err)",
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
                 (fec_short_now - fec_short_told) * 1000 / stream_rate,
                 fec_err_now - fec_err_told);
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
    fec_short_told = fec_short_now;
    fec_err_told = fec_err_now;
    short_frames_told = short_frames_now;

    /* Soak line, every 60 s, ahead of the streaming check below: if audio
     * has stopped, that is when the heap and the counters matter most.
     * Totals, not rates -- see the hub's copy. */
    static int health_left;
    if (--health_left <= 0) {
        health_left = 12;                      /* 12 x 5 s */
        hw_drift = uxTaskGetStackHighWaterMark(NULL);
        /* `window` is the lowest this minute, `min` the lowest since boot.
         * The pair dates a dip to this line, which the watermark alone
         * never could. Taken and cleared, like every windowed counter. */
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
                      " | wifi-drops %" PRIu32
                      " | alloc-fail %" PRIu32
                      " | clock %s (tsf %" PRIu32 "/probe %" PRIu32
                      ", wide-span %" PRIu32 ")"
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
                 n_anchor_late + n_anchor_soon, n_wifi_drops, n_alloc_fail,
                 tsf_fresh(esp_timer_get_time(), NULL) ? "TSF" : "probe",
                 n_tsf_used, n_tsf_fallback, n_tsf_wide,
                 n_phase_drop, n_short_reads, n_short_frames,
                 dma_starve_count(), n_gap_short_resyncs,
                 n_seq_dropped, n_decode_err, n_recv_err,
                 visualiser_source_name(), visualiser_hop(),
                 n_frames_rx, n_frames_bad);

        /*
         * Its own line for the same reason MEM has one -- see the hub's copy.
         *
         * This is what says the fine rate trim is running at all. `trim` is
         * what the servo asked for; the two totals are what playback actually
         * did about it, which is the only pair that can disagree. Expect one
         * frame per ~1.6 s in one direction at ~14 ppm of real drift; flat
         * means the trim is off, and both climbing means it is hunting across
         * zero. See n_trim_drops.
         *
         * The catch-up pair beside them is the same instrument for the
         * large-error drain: flat while |phase| stays under CATCHUP_ARM_US,
         * a burst of a few seconds when a knock is being paid off. They are
         * NOT part of the trim's frames/s arithmetic -- a drain deliberately
         * exceeds any rate the trim could claim -- which is why they get
         * their own keys rather than riding the totals above.
         */
        ESP_LOGW(TAG, "TRIM: %+ld Hz | dropped %" PRIu32 " dup %" PRIu32
                      " frames | catchup-drops %" PRIu32 " catchup-dups %"
                      PRIu32 " | retunes %" PRIu32 " coarse | volume %u/%d "
                      "vol-rx %" PRIu32,
                 (long)rate_trim_hz, n_trim_drops, n_trim_dups,
                 n_catchup_drops, n_catchup_dups, n_retunes,
                 audio_volume, AUDIO_VOL_MAX, n_vol_rx);

        /* Its own line rather than four more fields above -- see the hub's
         * copy for why, and for what the two pools mean. Here they should
         * read the same: no PSRAM on this board. */
        ESP_LOGW(TAG, "MEM: internal %u free (min %u, window %" PRIu32
                      ", largest %u) | total %" PRIu32 " (largest %u)",
                 (unsigned)heap_caps_get_free_size(CAP_USABLE_INTERNAL),
                 (unsigned)heap_caps_get_minimum_free_size(CAP_USABLE_INTERNAL),
                 heap_int_win == UINT32_MAX ? 0 : heap_int_win,
                 (unsigned)heap_caps_get_largest_free_block(CAP_USABLE_INTERNAL),
                 esp_get_free_heap_size(),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));

#if CONFIG_DANCEFLOOR_WIFI_LOGS
        /* The structured twin of the HEALTH line, for the collector's CSV.
         * The MEM figures are console-only -- see the hub's copy.
         * Every field is already in scope here; the role aliases are
         * documented on health_msg_t in sync_proto.h. */
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
