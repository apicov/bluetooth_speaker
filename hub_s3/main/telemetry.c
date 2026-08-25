/**
 * @file telemetry.c
 * @brief The periodic reporting, and the allocation hook that feeds it.
 *
 * Everything here runs on ring_monitor_task's 5 s tick, ahead of the servo:
 * telemetry_tick() reports before servo_tick() decides anything, so a stopped
 * stream -- which makes servo_tick() return immediately -- still gets its heap
 * and counter lines. The HEALTH, TRIM and MEM lines print once a minute; the
 * task and allocation failures print as soon as their counters move.
 */
#include "hub.h"

/**
 * @brief Record an allocation failure; the heap's own failure callback.
 *
 * Records only. IDF marks heap_caps_alloc_failed() HEAP_IRAM_ATTR because the
 * heap is usable with the flash cache disabled, so this is reachable from an
 * ISR or from a task running across a flash write -- hence IRAM_ATTR, no
 * string literals, and nothing called that is not itself in IRAM. Logging is
 * out for a second reason: this runs inside the allocator and ESP_LOGx
 * allocates. telemetry_tick() says it within 5 s from a context where saying
 * things is safe.
 *
 * @param size           Bytes the caller asked for.
 * @param caps           MALLOC_CAP_* the caller asked for.
 * @param function_name  Where it failed; unused, the counters carry no names.
 */
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
    /* Aged here as well as before each audio send, so a satellite that
     * vanishes while the stream is stopped still leaves the send list. */
    clients_age(esp_timer_get_time());

    const uint32_t heap_now = esp_get_free_heap_size();
    if (heap_now < heap_min_window) {
        heap_min_window = heap_now;
    }
    const uint32_t heap_int_now = (uint32_t)heap_caps_get_free_size(CAP_USABLE_INTERNAL);
    if (heap_int_now < heap_int_window) {
        heap_int_window = heap_int_now;
    }

    /* Repeated on a cadence, and through the WiFi log relay, because
     * task_start()'s own ESP_LOGE fires at boot -- before the relay exists and
     * on a console nobody is necessarily attached to. Its own line rather than
     * a health_msg_t field: HEALTH already runs past LOG_MSG_MAX and reaches
     * the collector cut off mid-word, so anything appended there is
     * console-only. */
    static uint32_t task_fail_told;
    if (n_task_fail != task_fail_told) {
        task_fail_told = n_task_fail;
        ESP_LOGE(TAG, "CRIPPLED: %" PRIu32 " task(s) failed to start: %s",
                 n_task_fail, s_task_fail_names);
    }

    /*
     * Said within 5 s of the fact rather than at the next HEALTH line: an
     * allocation failure is wanted next to whatever else the console was
     * saying at the time. The running count stays on HEALTH.
     *
     * Both pools, with the caps decoded. Whole-heap alone reads 8 MB free on a
     * PSRAM board beside a failed 1700-byte request -- true, and it says the
     * unit is fine at the one moment it is not. `min` is the load-bearing
     * figure of the four: the live figures are sampled HERE, up to 5 s after
     * the failure, by which time the condition has passed, while the watermark
     * is maintained continuously and still holds the dip.
     *
     * Snapshotting the live figures inside on_alloc_failed() instead is not
     * available. heap_caps_get_free_size, _get_info and
     * _get_largest_free_block carry no HEAP_IRAM_ATTR and are absent from the
     * heap component's linker.lf, so all three live in flash -- and the hook
     * is IRAM_ATTR precisely because an allocation can fail from an ISR
     * running with the flash cache disabled.
     */
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

    /* Totals rather than per-window rates: everything else here is cleared
     * every window, which answers "what is happening now" and cannot answer
     * "has this been happening slowly for an hour". */
    static int health_left;
    if (--health_left <= 0) {
        health_left = 12;                      /* 12 x 5 s */
        hw_mon = uxTaskGetStackHighWaterMark(NULL);

        /* `window` is the lowest this minute, `min` the lowest since boot. The
         * pair is the point: a window far below the current free figure dates
         * the dip to this line, which the watermark alone cannot. Taken and
         * cleared, like every other windowed counter. */
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

        /*
         * Its own line, short enough to survive the collector's LOG_MSG_MAX.
         *
         * This is what says the fine rate trim is running at all: `trim` is
         * what the servo asked for and the two frame totals are what playback
         * did about it, which is the only pair that can disagree. Flat means
         * the trim is off; both climbing means it is hunting across zero.
         *
         * The catch-up pair is the same instrument for the large-error drain
         * (audio_shift.h), and deliberately its own keys rather than part of
         * the totals -- a drain exceeds any rate the trim's arithmetic could
         * explain.
         */
        ESP_LOGW(TAG, "TRIM: %+ld Hz | dropped %" PRIu32 " dup %" PRIu32
                      " frames | catchup-drops %" PRIu32 " catchup-dups %"
                      PRIu32 " | retunes %" PRIu32 " coarse | volume %u/%d "
                      "vol-tx %" PRIu32,
                 (long)rate_trim_hz, n_trim_drops, n_trim_dups,
                 n_catchup_drops, n_catchup_dups, n_retunes,
                 audio_volume, AUDIO_VOL_MAX, n_vol_tx);

        /* Its own line for the same reason TRIM has one. The HEALTH heap figure
         * is whole-heap, which on this board is mostly PSRAM and cannot refuse
         * a WiFi buffer; this is the pool that can. On a board without PSRAM
         * the two agree, and that agreement is itself worth seeing -- it is
         * what says the figures read a real pool. */
        ESP_LOGW(TAG, "MEM: internal %u free (min %u, window %" PRIu32
                      ", largest %u) | total %" PRIu32 " (largest %u)",
                 (unsigned)heap_caps_get_free_size(CAP_USABLE_INTERNAL),
                 (unsigned)heap_caps_get_minimum_free_size(CAP_USABLE_INTERNAL),
                 heap_int_win == UINT32_MAX ? 0 : heap_int_win,
                 (unsigned)heap_caps_get_largest_free_block(CAP_USABLE_INTERNAL),
                 esp_get_free_heap_size(),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));

#if CONFIG_DANCEFLOOR_WIFI_LOGS
        /*
         * The structured twin of the HEALTH line, for the collector's CSV. It
         * does not carry the MEM figures: growing health_msg_t means editing
         * its size assertion in test_sync_proto.c and the struct format in
         * collect.py in lockstep, and probe.c relays satellite health with
         * sizeof(*m) rather than the received length, so a version skew
         * between units would be silent.
         *
         * The role-aliased fields carry this unit's counters:
         * reanchors_or_restarts = restarts, gaps_or_sta_left = sta-left,
         * wifi_drops_or_oversize = wifi-over, and the tail three are
         * sta-dropped / sta-nolease / sta-timeout. See health_msg_t.
         */
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

/* Registered before anything else allocates, so a failure during WiFi or
 * socket setup is caught too -- that is the phase with the largest single
 * requests. Declared in hub.h. */
void telemetry_register_alloc_hook(void)
{
    ESP_ERROR_CHECK(heap_caps_register_failed_alloc_callback(on_alloc_failed));
}
