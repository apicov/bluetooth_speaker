#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/i2s_std.h"

#include "sync_proto.h"
#include "sat.h"

const char *TAG = "sat";

int sock = -1;
sync_est_t est;
StreamBufferHandle_t ring;
i2s_chan_handle_t i2s_tx;
volatile int64_t stream_start_local;
volatile int64_t anchor_at;
volatile uint32_t stream_rate = 44100;
uint32_t tx_rate = 44100;
volatile int32_t rate_trim_hz;
volatile uint8_t audio_volume;
volatile bool audio_vol_known;
volatile uint32_t n_vol_rx;
int64_t stream_offset;
int64_t offset_slew_last;
volatile int32_t marker_sample = -1;
volatile int32_t samples_in;
phase_pt_t phase_q[PHASE_Q_LEN];
volatile uint32_t phase_head, phase_tail;
volatile int32_t phase_err_us;
volatile bool phase_valid;
sync_phase_hist_t phase_hist;
volatile int32_t restart_pos = -1;
volatile bool phase_stepped;
volatile int32_t step_report_mag;
volatile int32_t step_report_from, step_report_to;
volatile int32_t step_report_ring, step_report_trim;
volatile uint32_t step_report_pad;
volatile bool    step_report_pending;

volatile int32_t splice_report_us;
volatile int32_t splice_report_phase;
volatile int32_t splice_report_alt;
volatile bool    splice_report_pending;
tsf_reading_t tsf;
volatile uint32_t n_tsf_read_fail;
volatile uint32_t n_tsf_used;
volatile uint32_t n_tsf_fallback;
volatile uint32_t n_underruns;
volatile uint32_t n_reanchors;
volatile uint32_t n_splices;
volatile uint32_t n_retunes;
volatile uint32_t n_retunes_bad;
volatile uint32_t n_gaps;
volatile uint32_t n_fec_recovered;
volatile uint32_t n_fec_lost;
volatile uint32_t n_fec_holds;
volatile uint32_t n_fec_parity_rx;
volatile uint32_t n_fec_bad;
volatile uint32_t n_gap_short_resyncs;
volatile uint32_t n_seq_dropped;
volatile uint32_t n_decode_err;
volatile uint32_t n_recv_err;
volatile uint32_t n_wifi_drops;
volatile uint32_t n_wifi_lease_fail;
volatile uint32_t n_gap_frames;
volatile uint32_t n_gap_short;
volatile uint32_t n_gap_short_frames;
volatile uint32_t n_ring_full;
volatile uint32_t n_gap_resyncs;
volatile uint32_t n_anchor_upgrades;
volatile bool resync_request;
volatile bool anchor_provisional;
volatile bool     self_muted;
volatile uint32_t n_self_mutes;
volatile uint32_t n_self_retries;
volatile uint32_t n_anchor_late;
volatile uint32_t n_anchor_soon;
volatile uint32_t n_phase_drop;
volatile uint32_t n_short_reads;
volatile uint32_t n_short_frames;
volatile uint32_t n_audio_rx;
volatile int32_t  rx_gap_max_us;
volatile uint32_t rx_burst_max;
volatile int32_t  rx_lead_min_us = ARRIVAL_UNSEEN;
volatile uint32_t n_lead_insane;
volatile int32_t  ring_low_ms    = ARRIVAL_UNSEEN;
volatile int32_t  fec_hold_max_us;
volatile uint32_t n_trim_drops;
volatile uint32_t n_trim_dups;
volatile int32_t  catchup_frames;
volatile uint32_t n_catchup_drops;
volatile uint32_t n_catchup_dups;
volatile uint32_t n_tsf_wide;
volatile int64_t wifi_down_at;
volatile int64_t rejoined_at;
volatile int64_t est_newest_at;
volatile uint32_t n_frames_rx;
volatile uint32_t n_frames_bad;
volatile uint32_t hw_play;
volatile uint32_t hw_drift;
volatile uint32_t heap_min_window = UINT32_MAX;
volatile uint32_t heap_int_window = UINT32_MAX;
volatile uint32_t n_alloc_fail;
volatile uint32_t alloc_fail_size;
volatile uint32_t alloc_fail_caps;
volatile uint32_t n_task_fail;
char s_task_fail_names[64];
bool    s_refill_active;
int32_t s_refill_frames;
volatile int32_t retune_phase_before;
volatile bool    retune_watch;
volatile int64_t retune_outage_us;
volatile int64_t retune_done_at;
volatile uint8_t retune_tail_left;
volatile bool retuning;
volatile bool playing;
