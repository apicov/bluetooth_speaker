/*
 * Definitions for everything sat.h declares.
 *
 * Deliberately just the definitions: the reasoning, the measurements and the
 * ownership rules live in sat.h beside the declarations, which is where a
 * reader looks for them. Keeping them apart from the initialisers means there
 * is one place to change when a field changes hands, not two.
 */
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
uint32_t tx_rate = 44100;  /* what the output clock is actually set to */
volatile int32_t rate_trim_hz;  /* the fine correction playback applies instead */
volatile uint8_t audio_volume;  /* meaningless until audio_vol_known */
volatile bool audio_vol_known;  /* sticky: set by the first MSG_VOL, never cleared */
volatile uint32_t n_vol_rx;  /* MSG_VOL taken, repeats included */
int64_t stream_offset;
int64_t offset_slew_last;  /* when the slew last moved it */
volatile int32_t marker_sample = -1;  /* ring position of a tagged packet */
volatile int32_t samples_in;  /* frames written into the ring */
phase_pt_t phase_q[PHASE_Q_LEN];
volatile uint32_t phase_head, phase_tail;
volatile int32_t phase_err_us;  /* + = playing late */
volatile bool phase_valid;
sync_phase_hist_t phase_hist;
volatile int32_t restart_pos = -1;  /* ring position of a track boundary */
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
volatile uint32_t n_tsf_used;  /* anchors that used TSF */
volatile uint32_t n_tsf_fallback;  /* anchors that fell back */
volatile uint32_t n_underruns;  /* playback ran dry */
volatile uint32_t n_reanchors;  /* streams anchored, first included */
volatile uint32_t n_splices;  /* track-boundary corrections applied */
volatile uint32_t n_retunes;
volatile uint32_t n_retunes_bad;
volatile uint32_t n_gaps;  /* lost-packet gaps filled with silence */
volatile uint32_t n_fec_recovered;  /* lost packets rebuilt whole from parity */
volatile uint32_t n_fec_lost;  /* holes parity could not close */
volatile uint32_t n_fec_holds;  /* times packets were held waiting for a parity */
volatile uint32_t n_fec_parity_rx;  /* parity datagrams received */
volatile uint32_t n_fec_bad;  /* parity that arrived and could not be trusted */
volatile uint32_t n_gap_short_resyncs;  /* re-anchors forced by a fill that did not fit */
volatile uint32_t n_seq_dropped;  /* packets older than expected, dropped */
volatile uint32_t n_decode_err;  /* live-stream SBC frames that would not decode */
volatile uint32_t n_recv_err;  /* recvfrom() errors */
volatile uint32_t n_wifi_drops;  /* disconnects from the hub's AP */
volatile uint32_t n_wifi_lease_fail;  /* associations that never got an address */
volatile uint32_t n_gap_frames;  /* silence actually inserted; a repaired gap adds none */
volatile uint32_t n_gap_short;  /* gap fills the ring could not take */
volatile uint32_t n_gap_short_frames;
volatile uint32_t n_ring_full;  /* decoded blocks dropped, ring full */
volatile uint32_t n_gap_resyncs;  /* gaps too large to fill, re-anchored */
volatile uint32_t n_anchor_upgrades;  /* provisional anchors replaced */
volatile bool resync_request;
volatile bool anchor_provisional;
volatile uint32_t n_anchor_late;  /* anchors refused, play_at already past */
volatile uint32_t n_anchor_soon;  /* anchors refused, one just happened */
volatile uint32_t n_phase_drop;
volatile uint32_t n_short_reads;
volatile uint32_t n_short_frames;
volatile uint32_t n_audio_rx;
volatile int32_t  rx_gap_max_us;
volatile uint32_t rx_burst_max;
volatile int32_t  rx_lead_min_us = ARRIVAL_UNSEEN;
volatile uint32_t n_lead_insane;
volatile int32_t  ring_low_ms    = ARRIVAL_UNSEEN;
volatile uint32_t n_trim_drops;
volatile uint32_t n_trim_dups;
volatile int32_t  catchup_frames;
volatile uint32_t n_catchup_drops;
volatile uint32_t n_catchup_dups;
volatile uint32_t n_tsf_wide;
volatile int64_t wifi_down_at;
volatile int64_t rejoined_at;  /* 0 = the next anchor is not the first */
volatile int64_t est_newest_at;  /* when the newest probe landed */
volatile uint32_t n_frames_rx;  /* analysis frames taken from the hub */
volatile uint32_t n_frames_bad;  /* ... and rejected, wrong size */
volatile uint32_t hw_play;  /* stack headroom, sampled in-task */
volatile uint32_t hw_drift;
volatile uint32_t heap_min_window = UINT32_MAX;
volatile uint32_t heap_int_window = UINT32_MAX;
volatile uint32_t n_alloc_fail;
volatile uint32_t alloc_fail_size;  /* the largest request that failed */
volatile uint32_t alloc_fail_caps;
volatile uint32_t n_task_fail;
char s_task_fail_names[64];
bool    s_refill_active;
int32_t s_refill_frames;
volatile int32_t retune_phase_before;
volatile bool    retune_watch;  /* playback reports the next reading */
volatile int64_t retune_outage_us;
volatile int64_t retune_done_at;
volatile uint8_t retune_tail_left;
volatile bool retuning;
volatile bool playing;  /* the play task is feeding the DAC; read from the ISR */
