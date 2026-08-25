#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/i2s_std.h"
#include "esp_attr.h"
#include "sdkconfig.h"

#include "sync_proto.h"
#include "audio_shift.h"

extern const char *TAG;

#define AP_SSID    CONFIG_DANCEFLOOR_AP_SSID
#define AP_PASS    CONFIG_DANCEFLOOR_AP_PASS
#define MASTER_IP  "192.168.4.1"

#define PROBE_PERIOD_MS 250

#define RING_BYTES  (CONFIG_DANCEFLOOR_RING_KB * 1024)

extern int sock;
extern sync_est_t est;
extern StreamBufferHandle_t ring;
extern i2s_chan_handle_t i2s_tx;

extern volatile int64_t stream_start_local;

extern volatile int64_t anchor_at;
extern volatile uint32_t stream_rate;
extern uint32_t tx_rate;

extern int64_t stream_offset;
extern int64_t offset_slew_last;

extern volatile int32_t marker_sample;
extern volatile int32_t samples_in;

#define PHASE_Q_LEN 32
typedef struct {
    int32_t pos;
    int64_t play_at;
} phase_pt_t;

extern phase_pt_t phase_q[PHASE_Q_LEN];
extern volatile uint32_t phase_head, phase_tail;
extern volatile int32_t phase_err_us;
extern volatile bool phase_valid;

extern sync_phase_hist_t phase_hist;
extern volatile int32_t restart_pos;

extern volatile bool phase_stepped;

#define MAX_SPLICE_MS 150

#define SPLICE_INSERT_HEADROOM_MS 50

#define PHASE_INSANE_US 1000000

#define ANCHOR_MIN_LEAD_US     125000
#define ANCHOR_MIN_INTERVAL_US 1000000
#define ANCHOR_GIVE_UP_US      5000000

#define GAP_RESYNC_MS 150

#define DEPTH_NET_HOLD_US      20000000

extern volatile int32_t step_report_mag;
extern volatile int32_t step_report_from, step_report_to;
extern volatile int32_t step_report_ring, step_report_trim;
extern volatile uint32_t step_report_pad;
extern volatile bool    step_report_pending;

extern volatile int32_t splice_report_us;
extern volatile int32_t splice_report_phase;

extern volatile int32_t splice_report_alt;
extern volatile bool    splice_report_pending;

#define TSF_READ_TRIES 8
typedef struct {
    volatile uint32_t seq;
    volatile int64_t  offset_us;
    volatile int64_t  at;
} tsf_reading_t;

extern tsf_reading_t tsf;

static inline void tsf_publish(int64_t offset_us, int64_t at)
{
    tsf.seq++;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    tsf.offset_us = offset_us;
    tsf.at = at;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    tsf.seq++;
}

extern volatile uint32_t n_tsf_read_fail;

static inline bool tsf_read(int64_t *offset_us, int64_t *at)
{
    for (int t = 0; t < TSF_READ_TRIES; t++) {
        const uint32_t s0 = tsf.seq;
        if (s0 & 1u) {
            continue;
        }
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        const int64_t us = tsf.offset_us;
        const int64_t a  = tsf.at;
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        if (s0 == tsf.seq) {
            *offset_us = us;
            *at = a;
            return true;
        }
    }
    n_tsf_read_fail++;
    return false;
}

extern volatile uint32_t n_tsf_used;
extern volatile uint32_t n_tsf_fallback;

#define TSF_MAX_AGE_US 1000000

static inline bool tsf_fresh(int64_t now, int64_t *offset_us)
{
    int64_t us, at;
    if (!tsf_read(&us, &at)) {
        return false;
    }
    if (at && now - at < TSF_MAX_AGE_US) {
        if (offset_us) *offset_us = us;
        return true;
    }
    return false;
}

extern volatile uint32_t n_underruns;
extern volatile uint32_t n_reanchors;
extern volatile uint32_t n_splices;
extern volatile uint32_t n_retunes;
extern volatile uint32_t n_retunes_bad;

extern volatile uint32_t n_gaps;

extern volatile uint32_t n_fec_recovered;
extern volatile uint32_t n_fec_lost;
extern volatile uint32_t n_fec_holds;
extern volatile uint32_t n_fec_parity_rx;
extern volatile uint32_t n_fec_bad;

extern volatile uint32_t n_gap_short_resyncs;
extern volatile uint32_t n_seq_dropped;
extern volatile uint32_t n_decode_err;
extern volatile uint32_t n_recv_err;
extern volatile uint32_t n_wifi_drops;

extern volatile uint32_t n_wifi_lease_fail;

extern volatile uint32_t n_gap_frames;
extern volatile uint32_t n_gap_short;
extern volatile uint32_t n_gap_short_frames;
extern volatile uint32_t n_ring_full;
extern volatile uint32_t n_gap_resyncs;
extern volatile uint32_t n_anchor_upgrades;

extern volatile bool resync_request;

extern volatile bool anchor_provisional;
extern volatile uint32_t n_anchor_late;
extern volatile uint32_t n_anchor_soon;

extern volatile uint32_t n_phase_drop;

extern volatile uint32_t n_short_reads;
extern volatile uint32_t n_short_frames;

#define ARRIVAL_UNSEEN INT32_MAX
extern volatile uint32_t n_audio_rx;
extern volatile int32_t  rx_gap_max_us;
extern volatile uint32_t rx_burst_max;
extern volatile int32_t  rx_lead_min_us;
extern volatile uint32_t n_lead_insane;
extern volatile int32_t  ring_low_ms;

extern volatile int32_t  fec_hold_max_us;

#define RX_BURST_US 2000

#define LEAD_INSANE_US PHASE_INSANE_US

#define TSF_SPAN_MAX_US 100
extern volatile uint32_t n_tsf_wide;

extern volatile int64_t wifi_down_at;
extern volatile int64_t rejoined_at;
extern volatile int64_t est_newest_at;
extern volatile uint32_t n_frames_rx;
extern volatile uint32_t n_frames_bad;
extern volatile uint32_t hw_play;
extern volatile uint32_t hw_drift;

extern volatile uint32_t heap_min_window;

#define CAP_USABLE_INTERNAL (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)

extern volatile uint32_t heap_int_window;
extern volatile uint32_t n_alloc_fail;
extern volatile uint32_t alloc_fail_size;
extern volatile uint32_t alloc_fail_caps;

extern volatile uint32_t n_task_fail;
extern char s_task_fail_names[64];

#define RING_TARGET_MS 350

#define REFILL_FAST_US 1000
extern bool    s_refill_active;
extern int32_t s_refill_frames;

extern volatile int32_t rate_trim_hz;

extern volatile uint8_t audio_volume;

extern volatile bool audio_vol_known;

extern volatile uint32_t n_vol_rx;

extern volatile uint32_t n_trim_drops;
extern volatile uint32_t n_trim_dups;

extern volatile int32_t  catchup_frames;
extern volatile uint32_t n_catchup_drops;
extern volatile uint32_t n_catchup_dups;

extern volatile int32_t retune_phase_before;
extern volatile bool    retune_watch;
extern volatile int64_t retune_outage_us;

extern volatile int64_t retune_done_at;
extern volatile uint8_t retune_tail_left;

extern volatile bool retuning;

extern volatile bool playing;

#define MUTE_WINDOW_US   3000000
#define MUTE_AUDIO_MIN   15

#define MUTE_RSSI_FLOOR  (-85)

#define MUTE_RSSI_REJOIN  (-80)
#define MUTE_REJOIN_TICKS 8

#define MUTE_RETRY_US   60000000

#define MUTE_TRIAL_US    5000000

#define MUTE_SLOTS       (MUTE_WINDOW_US / (PROBE_PERIOD_MS * 1000))

extern volatile bool     self_muted;
extern volatile uint32_t n_self_mutes;
extern volatile uint32_t n_self_retries;

void wifi_start_sta(void);
void socket_start(void);

void wifi_retry_tick(void);

void i2s_start(uint32_t rate);

uint32_t dma_starve_count(void);
void write_audio(const int16_t *frames, size_t n_frames, uint8_t vol);

void write_audio_reset_ramp(void);
void retune_output(uint32_t hz);
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
int64_t vis_master_to_local(int64_t master_us);
#endif

void probe_task(void *arg);
bool clock_offset(int64_t *out, bool *used_tsf);
void track_offset(void);

void rx_task(void *arg);

void play_task(void *arg);

void servo_tick(void);

void telemetry_tick(void);

void on_alloc_failed(size_t size, uint32_t caps, const char *function_name);
