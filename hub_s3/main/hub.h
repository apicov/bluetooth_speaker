
#pragma once

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_attr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "sdkconfig.h"

#include "audio_out.h"
#include "sync_proto.h"
#include "audio_shift.h"
#include "visualiser.h"
#include "wifi_log.h"

#include "streamer.h"

#define AP_SSID   CONFIG_DANCEFLOOR_AP_SSID
#define AP_PASS   CONFIG_DANCEFLOOR_AP_PASS

#define LEAD_US   350000

#define RESYNC_US  70000

#define RESYNC_HARD_US 300000

#define TIMELINE_HOLD_STARVE_MS   150
#define TIMELINE_HOLD_GIVE_UP_US  30000000

#define SOURCE_STALL_US    300000
#define SOURCE_STEADY_US   500000
#define SOURCE_GIVE_UP_US 5000000

#define TIMELINE_SLEW_US 20

#define TX_BACKOFF_US           40000

#define DTIM_HOLD_US 102400

#define TX_FRAME_PACE_US       DTIM_HOLD_US

#define TX_FRAME_BATCH         12

#define LOCAL_RING_BYTES (80 * 1024)

extern const char *TAG;

extern StreamBufferHandle_t local_ring;
extern i2s_chan_handle_t i2s_tx;
extern int sock;
extern volatile uint32_t sample_rate;
extern uint32_t tx_rate;
extern uint32_t rate_ema;

#if CONFIG_DANCEFLOOR_ENABLE_MARKER
extern volatile int64_t s_marker_at;
#endif

extern volatile int32_t s_marker_sample;
extern volatile int32_t s_samples_in;

#define PHASE_Q_LEN 32
typedef struct {
    int32_t pos;
    int64_t play_at;
} phase_pt_t;

extern phase_pt_t s_phase_q[PHASE_Q_LEN];
extern volatile uint32_t s_phase_head, s_phase_tail;
extern volatile int32_t s_phase_err_us;
extern volatile bool s_phase_valid;

extern sync_phase_hist_t s_phase_hist;

extern volatile int32_t s_phase_med_us;
extern volatile bool s_phase_med_valid;

extern volatile bool s_phase_stepped;
extern volatile bool s_restart_pending;

extern uint8_t s_jump_arm;

extern volatile bool s_underrun_recover;
extern volatile int32_t s_restart_pos;

#define MAX_SPLICE_MS 150

#define SPLICE_INSERT_HEADROOM_MS 50

extern bool s_slewing;
extern bool s_slew_told;
extern int64_t s_slew_since;

extern volatile bool retuning;

extern volatile int64_t local_start;
extern volatile uint32_t local_epoch;

extern volatile bool s_playing;

extern volatile uint32_t s_feed_dropped;
extern volatile uint32_t s_tx_fail;

extern volatile uint32_t s_audio_pkts;

typedef enum {
    TX_LANE_AUDIO = 0,
    TX_LANE_FRAME,
    TX_LANE_VOL,
    TX_LANE_META,
    TX_LANE_PROBE,
    TX_LANE_FEC,
    TX_LANE_N,
} tx_lane_t;

extern volatile uint32_t s_tx_lane_fail[TX_LANE_N];

#define TX_NEAR_US              5000
extern volatile int64_t  s_tx_frame_sent_us;
extern volatile uint32_t n_refuse_near_frame;

extern volatile uint32_t n_audio_retry;
extern volatile uint32_t n_audio_retry_ok;

extern volatile uint32_t n_join_churn;

extern volatile uint32_t n_tx_cong_skip;
extern volatile int64_t s_tx_congested_until;

extern volatile uint32_t n_tx_pace_skip;

extern volatile int32_t n_fanout_gap_max_us;

#define LEAD_UNSEEN INT32_MAX

#define LEAD_INSANE_US 1000000
extern volatile int32_t n_lead_min_us;

void tx_fail_note(tx_lane_t lane, int err);

void tx_fail_note_audio(int err);
void tx_fail_summary(char *buf, size_t len);

void tx_fail_lanes(char *buf, size_t len);

void tx_burst_summary(char *buf, size_t len);

void tx_air_summary(char *buf, size_t len);

void tx_send_ok(void);

uint32_t dma_starve_count(void);

extern volatile uint32_t n_underruns;
extern volatile uint32_t n_restarts;
extern volatile uint32_t n_splices;
extern volatile uint32_t n_retunes;
extern volatile uint32_t n_retunes_bad;
extern volatile uint32_t n_sta_left;
extern volatile uint32_t hw_play;
extern volatile uint32_t hw_mon;

extern volatile uint32_t heap_min_window;

#define CAP_USABLE_INTERNAL (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)

extern volatile uint32_t heap_int_window;

extern volatile uint32_t n_alloc_fail;
extern volatile uint32_t alloc_fail_size;
extern volatile uint32_t alloc_fail_caps;

extern volatile uint32_t n_task_fail;
extern char s_task_fail_names[64];

extern volatile uint32_t n_phase_drop;

extern volatile uint32_t n_refill_withheld;
extern volatile uint32_t n_short_reads;
extern volatile uint32_t n_short_frames;

extern volatile uint32_t n_trim_drops;
extern volatile uint32_t n_trim_dups;

extern volatile int32_t  catchup_frames;
extern volatile uint32_t n_catchup_drops;
extern volatile uint32_t n_catchup_dups;

#define REFILL_FAST_US 1000
extern bool s_refill_active;
extern int32_t s_refill_frames;

#define MAX_CLIENTS 15

#define CLIENT_TIMEOUT_US 2000000

#define VOL_REPEAT_US     1000000
#define VOL_CHANGE_REPEATS 3

typedef struct {
    struct sockaddr_in addr;
    int64_t last_seen;
} client_t;

extern client_t s_clients[MAX_CLIENTS];
extern portMUX_TYPE s_clients_lock;
extern esp_netif_t *s_ap_netif;
extern volatile uint32_t n_sta_dropped;
extern volatile uint32_t n_sta_nolease;

extern volatile uint32_t n_sta_timeout;

extern uint32_t n_wifi_oversize;

extern volatile uint32_t n_fec_sent;
extern volatile uint32_t n_fec_skipped;
extern volatile uint32_t n_fec_cong_skip;

extern volatile int32_t s_vis_anchor_pos;
extern volatile int64_t s_vis_anchor_due;

extern int32_t s_pending_pos;

extern volatile int64_t s_sync_err_us;
extern volatile int64_t s_sync_at;

extern volatile int32_t s_hub_splice_us;
extern volatile int64_t s_hub_splice_at;

extern volatile int32_t s_hub_splice_alt_us;

#define RATE_SANE_MIN 8000
#define RATE_SANE_MAX 192000

extern volatile int32_t rate_trim_hz;

extern volatile uint8_t audio_volume;

extern volatile bool audio_vol_known;

extern volatile uint32_t n_vol_tx;

extern volatile int32_t s_retune_phase_before;
extern volatile bool    s_retune_watch;
extern volatile int64_t s_retune_outage_us;

extern volatile int64_t s_retune_done_at;
extern volatile uint8_t s_retune_tail_left;

void i2s_start(uint32_t rate);
void retune_dac(uint32_t hz);

void wifi_start_ap(void);
void socket_start(void);

void client_seen(const struct sockaddr_in *from);
void client_joined(const uint8_t mac[6], const esp_ip4_addr_t *ip);
void client_gone(const uint8_t mac[6]);

void clients_snapshot(client_t *dst);

void clients_age(int64_t now);

#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
void publish_frame(const vis_frame_t *f);
#endif

void local_play_task(void *arg);
void probe_task(void *arg);
void ring_monitor_task(void *arg);

void telemetry_tick(void);
void servo_tick(void);
void telemetry_register_alloc_hook(void);

#if CONFIG_DANCEFLOOR_ENABLE_MARKER

void marker_start(void);
#endif
