
#include "hub.h"

static IRAM_ATTR void on_alloc_failed(size_t size, uint32_t caps, const char *function_name)
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

    clients_age(esp_timer_get_time());

    const uint32_t heap_now = esp_get_free_heap_size();
    if (heap_now < heap_min_window) {
        heap_min_window = heap_now;
    }
    const uint32_t heap_int_now = (uint32_t)heap_caps_get_free_size(CAP_USABLE_INTERNAL);
    if (heap_int_now < heap_int_window) {
        heap_int_window = heap_int_now;
    }

    static uint32_t task_fail_told;
    if (n_task_fail != task_fail_told) {
        task_fail_told = n_task_fail;
        ESP_LOGE(TAG, "CRIPPLED: %" PRIu32 " task(s) failed to start: %s",
                 n_task_fail, s_task_fail_names);
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

    static int health_left;
    if (--health_left <= 0) {
        health_left = 12;
        hw_mon = uxTaskGetStackHighWaterMark(NULL);

        const uint32_t heap_win = heap_min_window;
        heap_min_window = UINT32_MAX;
        const uint32_t heap_int_win = heap_int_window;
        heap_int_window = UINT32_MAX;
        ESP_LOGW(TAG, "HEALTH: up %llu s | heap %" PRIu32 " (min %" PRIu32
                      ", window %" PRIu32 ", largest %u) | "
                      "stack play %" PRIu32 " mon %" PRIu32 " | underruns %" PRIu32
                      " restarts %" PRIu32 " splices %" PRIu32 " retunes %" PRIu32
                      " (%" PRIu32 " refused) | sta-left %" PRIu32
                      " (dropped %" PRIu32 ", no-lease %" PRIu32
                      ") | sta-timeout %" PRIu32
                      " | alloc-fail %" PRIu32
                      " | refill-withheld %" PRIu32
                      " | phase-drop %" PRIu32 " short-reads %" PRIu32
                      " (%" PRIu32 " frames) | wifi-over %" PRIu32
                      " | dma-starve %" PRIu32,
                 (unsigned long long)(esp_timer_get_time() / 1000000),
                 esp_get_free_heap_size(), esp_get_minimum_free_heap_size(),
                 heap_win == UINT32_MAX ? 0 : heap_win,
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT),
                 hw_play, hw_mon, n_underruns, n_restarts, n_splices,
                 n_retunes, n_retunes_bad, n_sta_left, n_sta_dropped,
                 n_sta_nolease, n_sta_timeout, n_alloc_fail,
                 n_refill_withheld,
                 n_phase_drop, n_short_reads, n_short_frames, n_wifi_oversize,
                 dma_starve_count());

        ESP_LOGW(TAG, "TRIM: %+ld Hz | dropped %" PRIu32 " dup %" PRIu32
                      " frames | catchup-drops %" PRIu32 " catchup-dups %"
                      PRIu32 " | retunes %" PRIu32 " coarse | volume %u/%d "
                      "vol-tx %" PRIu32,
                 (long)rate_trim_hz, n_trim_drops, n_trim_dups,
                 n_catchup_drops, n_catchup_dups, n_retunes,
                 audio_volume, AUDIO_VOL_MAX, n_vol_tx);

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
        h.role = LOG_ROLE_HUB;
        h.seq = health_seq++;
        h.uptime_s = (uint64_t)(esp_timer_get_time() / 1000000);
        h.heap_cur = esp_get_free_heap_size();
        h.heap_min = esp_get_minimum_free_heap_size();
        h.heap_win = heap_win == UINT32_MAX ? 0 : heap_win;
        h.heap_largest = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
        h.hw_play = hw_play;
        h.hw_mon = hw_mon;
        h.underruns = n_underruns;
        h.reanchors_or_restarts = n_restarts;
        h.splices = n_splices;
        h.retunes = n_retunes;
        h.retunes_refused = n_retunes_bad;
        h.gaps_or_sta_left = n_sta_left;
        h.wifi_drops_or_oversize = n_wifi_oversize;
        h.alloc_fail = n_alloc_fail;
        h.phase_drop = n_phase_drop;
        h.short_reads = n_short_reads;
        h.short_frames = n_short_frames;
        h.ring_full_or_sta_dropped = n_sta_dropped;
        h.upgrades_or_sta_nolease = n_sta_nolease;
        h.anchors_refused_or_timeout = n_sta_timeout;
        h.log_dropped = wifi_log_dropped();
        h.log_no_dest = wifi_log_no_dest();
        wifi_log_send_to_dest(&h, sizeof h);
#endif
    }
}

void telemetry_register_alloc_hook(void)
{
    ESP_ERROR_CHECK(heap_caps_register_failed_alloc_callback(on_alloc_failed));
}
