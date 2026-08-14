/*
 * All the periodic reporting, and the allocation hook that feeds it.
 *
 * This ran inside ring_monitor_task, interleaved with the servo, for the stated
 * and good reason that it is a task which can afford to block on a UART. Good
 * reason, wrong room -- roughly 170 of that function's 318 lines had nothing to
 * do with the rate control it was named for. Same split the satellite made
 * between servo.c and telemetry.c, for the same reason.
 *
 * Ordering is preserved exactly: telemetry_tick() runs BEFORE servo_tick() on
 * every 5 s pass, because if audio has stopped that is when the heap and the
 * counters matter most.
 */
#include "hub.h"

/*
 * Records only, and lives in IRAM. Both are load-bearing.
 *
 * IDF marks heap_caps_alloc_failed() HEAP_IRAM_ATTR precisely because the heap
 * is usable with the flash cache disabled, so this can be reached from an ISR
 * or from a task running while a flash write is in progress. A hook in flash
 * would fault there -- a diagnostic for running out of memory that crashes
 * under the one condition it exists to observe. Hence IRAM_ATTR, no string
 * literals, and nothing called that is not itself in IRAM.
 *
 * And no logging even where it would be reachable: this runs inside the
 * allocator, and ESP_LOGx allocates. ring_monitor_task says it within 5 s from
 * a context where saying things is safe.
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

/* The 5 s reporting pass: the first half of what ring_monitor_task's loop body
 * used to be. health_left, alloc_fail_told and health_seq are still the
 * function-local statics they were there -- same storage, same lifetime. */

void telemetry_tick(void)
{
    /* Age the send list even when nothing is playing. Aging used to happen only
     * inside the audio send loop, so a satellite that vanished while the stream
     * was stopped stayed on the list until audio resumed. */
    clients_age(esp_timer_get_time());

    const uint32_t heap_now = esp_get_free_heap_size();
    if (heap_now < heap_min_window) {
        heap_min_window = heap_now;
    }
    const uint32_t heap_int_now = (uint32_t)heap_caps_get_free_size(CAP_USABLE_INTERNAL);
    if (heap_int_now < heap_int_window) {
        heap_int_window = heap_int_now;
    }

    /*
     * Tasks that failed to start, said out loud.
     *
     * task_start() has counted these and accumulated their names since it was
     * written, and until now NOTHING READ EITHER -- n_task_fail appeared on no
     * log line and in no health_msg_t field, and s_task_fail_names was a 64-byte
     * buffer that was appended to and never printed. The satellite reads both at
     * its GOT_IP handler and prints exactly this line; the hub only ever had the
     * one-shot ESP_LOGE inside task_start.
     *
     * That ESP_LOGE is not redundant with this, and this is not a duplicate of
     * it. It fires at boot, on a console nobody is necessarily attached to yet,
     * and before the WiFi log relay exists -- so a unit that came up crippled
     * and was looked at five minutes later said nothing about it at all. This
     * repeats the fact on a cadence, through the relay, and names the tasks.
     *
     * Its own line rather than a HEALTH field, for the reason the MEM line is
     * also its own: HEALTH already runs past LOG_MSG_MAX and reaches the
     * collector cut off mid-word, so anything appended there is console-only.
     */
    static uint32_t task_fail_told;
    if (n_task_fail != task_fail_told) {
        task_fail_told = n_task_fail;
        ESP_LOGE(TAG, "CRIPPLED: %" PRIu32 " task(s) failed to start: %s",
                 n_task_fail, s_task_fail_names);
    }

    /* Said once, and within 5 s of the fact rather than at the next soak
     * line -- an allocation failure is the thing you want to see next to
     * whatever else the console was saying at the time. The running count
     * stays on HEALTH.
     *
     * Both pools, and the caps spelled out. This line used to report the
     * whole heap only, which on a PSRAM board reads 8 MB free beside a
     * failed 1700-byte request -- true, useless, and actively misleading:
     * it says the unit is fine at the one moment it is not. The caps were
     * raw hex and nothing decoded them, so 0x1800 had to be looked up by
     * hand while a floor was down.
     *
     * `min` is the load-bearing figure, and the reason the rest of the line
     * cannot be trusted on its own. These numbers are sampled HERE, up to
     * 5 s after the failure, by which time the condition has passed: the
     * first capture printed "internal 21912 free, largest 11776" beside a
     * failed 1700-byte request, which disproves the failure it accompanies.
     * The watermark is the one thing the allocator maintains continuously,
     * so it still holds the dip -- 1520 bytes, in that same capture.
     *
     * Do not try to snapshot the live figures inside the hook instead. The
     * obstacle is not locking: heap_caps_alloc_failed() runs after the
     * allocator has returned, holding nothing. It is that
     * heap_caps_get_free_size, _get_info and _get_largest_free_block carry
     * no HEAP_IRAM_ATTR and are absent from the heap component's linker.lf,
     * so all three live in flash -- and the hook is IRAM_ATTR precisely
     * because an allocation can fail from an ISR running with the flash
     * cache disabled. Calling them there is a panic, not a stall.
     *
     * Near the wire limit: 185 bytes with every field saturated against a
     * LOG_MSG_MAX of 192. `total` lost its largest-block figure to make
     * room for `min`, which is the right trade -- that pair is context, and
     * MEM carries it every 60 s anyway. */
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
     * Soak line, every 60 s, and deliberately ahead of the streaming check
     * below: if audio has stopped, that is exactly when the heap and the
     * counters matter most.
     *
     * Totals rather than rates. Everything else here is cleared every
     * window, which answers "what is happening now" and cannot answer "has
     * this been happening slowly for an hour" -- and the longest run this
     * system had ever been given was seven minutes.
     */
    static int health_left;
    if (--health_left <= 0) {
        health_left = 12;                      /* 12 x 5 s */
        hw_mon = uxTaskGetStackHighWaterMark(NULL);
        /* `window` is the lowest this minute, `min` the lowest since boot.
         * The pair is the point: a window far below the current free figure
         * dates the dip to this line, which the watermark alone never
         * could. Taken and cleared, like every other windowed counter. */
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
                      " | dma-starve %" PRIu32 " fec-trunc %" PRIu32,
                 (unsigned long long)(esp_timer_get_time() / 1000000),
                 esp_get_free_heap_size(), esp_get_minimum_free_heap_size(),
                 heap_win == UINT32_MAX ? 0 : heap_win,
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT),
                 hw_play, hw_mon, n_underruns, n_restarts, n_splices,
                 n_retunes, n_retunes_bad, n_sta_left, n_sta_dropped,
                 n_sta_nolease, n_sta_timeout, n_alloc_fail,
                 n_refill_withheld,
                 n_phase_drop, n_short_reads, n_short_frames, n_wifi_oversize,
                 dma_starve_count(), n_fec_truncated);

        /*
         * Its own line, not four more fields above. The HEALTH line already
         * runs past LOG_MSG_MAX and arrives at the collector cut off
         * mid-word, so anything appended there would be read on this
         * console and nowhere else.
         *
         * The heap figure above is whole-heap, which on this board is mostly
         * PSRAM and cannot refuse a WiFi buffer. This is the pool that can.
         * On a board without PSRAM the two agree, and that agreement is
         * itself worth being able to see -- it is how the satellite's copy
         * of this line proves the figures are reading a real pool rather
         * than printing a constant.
         */
        /*
         * Its own line for the same reason MEM has one, and short enough to
         * survive the collector.
         *
         * This is what says the fine rate trim is running at all. `trim` is
         * what the servo asked for; the two totals are what playback actually
         * did about it, which is the only pair that can disagree. Expect one
         * frame per ~1.6 s in one direction at ~14 ppm of real drift; flat
         * means the trim is off, and both climbing means it is hunting across
         * zero. See n_trim_drops.
         */
        ESP_LOGW(TAG, "TRIM: %+ld Hz | dropped %" PRIu32 " dup %" PRIu32
                      " frames | retunes %" PRIu32 " coarse",
                 (long)rate_trim_hz, n_trim_drops, n_trim_dups, n_retunes);

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
         * It does not carry the MEM figures: growing health_msg_t past its
         * pinned 108 bytes means editing the size assertion in
         * test_sync_proto.c and the struct format in collect.py in lockstep,
         * and the hub relays satellite health with sizeof(*m) rather than
         * the received length, so a version skew between units would be
         * silent. Read MEM on the console; revisit if a slow leak ever needs
         * plotting.
         * The role-aliased fields carry this unit's counters: reanchors_or_
         * restarts = restarts, gaps_or_sta_left = sta-left, wifi_drops_or_
         * oversize = wifi-over, and the tail three are sta-dropped /
         * sta-nolease / sta-timeout (see health_msg_t). */
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

/* Registered before anything else allocates, so a failure during WiFi or socket
 * setup is caught too -- that is the phase with the largest single requests. */
void telemetry_register_alloc_hook(void)
{
    ESP_ERROR_CHECK(heap_caps_register_failed_alloc_callback(on_alloc_failed));
}
