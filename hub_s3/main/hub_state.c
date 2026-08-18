/*
 * Definitions for the state hub.h declares.
 *
 * Nothing but definitions: every comment explaining what these mean, who writes
 * them and what a torn read costs is in hub.h, beside the declaration. Keeping
 * them apart is what lets hub.h be read as the ownership document it is.
 */
#include "hub.h"

const char *TAG = "stream";
StreamBufferHandle_t local_ring;
i2s_chan_handle_t i2s_tx;
int sock = -1;
volatile uint32_t sample_rate = 44100;
uint32_t tx_rate = 44100;
uint32_t rate_ema;
volatile int32_t rate_trim_hz;
volatile uint8_t audio_volume = AUDIO_VOL_MAX;
#if CONFIG_DANCEFLOOR_ENABLE_MARKER
volatile int64_t s_marker_at;
#endif
volatile int32_t s_marker_sample = -1;
volatile int32_t s_samples_in;
phase_pt_t s_phase_q[PHASE_Q_LEN];
volatile uint32_t s_phase_head, s_phase_tail;
volatile int32_t s_phase_err_us;
volatile bool s_phase_valid;
sync_phase_hist_t s_phase_hist;
volatile int32_t s_phase_med_us;
volatile bool s_phase_med_valid;
volatile bool s_phase_stepped;
volatile bool s_restart_pending;
uint8_t s_jump_arm;
volatile bool s_underrun_recover;
volatile int32_t s_restart_pos = -1;
bool s_slewing;
bool s_slew_told;
int64_t s_slew_since;
volatile bool retuning;
volatile int64_t local_start;
volatile uint32_t local_epoch;
volatile bool s_playing;
volatile uint32_t s_feed_dropped;
volatile uint32_t s_tx_fail;
volatile uint32_t s_tx_fail_audio;
volatile uint32_t s_audio_pkts;
volatile uint32_t n_tx_cong_skip;
volatile int64_t s_tx_congested_until;
volatile uint32_t n_tx_pace_skip;
volatile uint32_t n_underruns;
volatile uint32_t n_restarts;
volatile uint32_t n_splices;
volatile uint32_t n_retunes;
volatile uint32_t n_retunes_bad;
volatile uint32_t n_sta_left;
volatile uint32_t hw_play;
volatile uint32_t hw_mon;
volatile uint32_t heap_min_window = UINT32_MAX;
volatile uint32_t heap_int_window = UINT32_MAX;
volatile uint32_t n_alloc_fail;
volatile uint32_t alloc_fail_size;
volatile uint32_t alloc_fail_caps;
volatile uint32_t n_task_fail;
char s_task_fail_names[64];
volatile uint32_t n_phase_drop;
volatile uint32_t n_refill_withheld;
volatile uint32_t n_short_reads;
volatile uint32_t n_short_frames;
volatile uint32_t n_trim_drops;
volatile uint32_t n_trim_dups;
volatile int32_t  catchup_frames;
volatile uint32_t n_catchup_drops;
volatile uint32_t n_catchup_dups;
bool s_refill_active;
int32_t s_refill_frames;
client_t s_clients[MAX_CLIENTS];
portMUX_TYPE s_clients_lock = portMUX_INITIALIZER_UNLOCKED;
esp_netif_t *s_ap_netif;
volatile uint32_t n_sta_dropped;
volatile uint32_t n_sta_nolease;
volatile uint32_t n_sta_timeout;
uint32_t n_wifi_oversize;
uint32_t n_fec_truncated;
volatile int32_t s_vis_anchor_pos;
volatile int64_t s_vis_anchor_due;
int32_t s_pending_pos;
volatile int64_t s_sync_err_us;
volatile int64_t s_sync_at;
volatile int32_t s_hub_splice_us;
volatile int64_t s_hub_splice_at;
volatile int32_t s_hub_splice_alt_us;
volatile int32_t s_retune_phase_before;
volatile bool    s_retune_watch;
volatile int64_t s_retune_outage_us;
volatile int64_t s_retune_done_at;
volatile uint8_t s_retune_tail_left;
