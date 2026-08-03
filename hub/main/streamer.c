#include "streamer.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "nvs_flash.h"

#include "sync_proto.h"
#include "visualiser.h"

#define AP_SSID   "dancefloor"
#define AP_PASS   "dancefloor"

/*
 * How far ahead of playback each chunk is stamped. Must exceed worst-case
 * network delivery -- measured RTT peaked at 28 ms, so this is a ~7x margin.
 *
 * Reduced from 250 ms so that LEAD + RESYNC stays inside the satellite's 64 kB
 * ring (372 ms): with bursty input the actual lead swings around this value
 * rather than sitting on it.
 */
#define LEAD_US   200000

/*
 * How far the presentation timeline may wander from real time before restarting.
 *
 * 120 ms, because SBC over UART is bursty. A2DP packets arrive ~23/s, each
 * carrying ~43 ms of audio that decodes in one go, so the timeline legitimately
 * races ahead while a burst is consumed and falls behind while waiting for the
 * next -- measured swings of +-75 ms. The old 50 ms threshold fired several
 * times a second.
 *
 * The I2S link used to hide this by pacing the audio; UART does not.
 *
 * This does not affect the local ring, which is governed by rate rather than by
 * the timeline. It does set how far a satellite's start time can be off, so
 * LEAD + RESYNC must fit the satellite ring: 200 + 120 = 320 ms against 372 ms.
 */
#define RESYNC_US 120000

/* Local playback ring. The master delays its own audio by LEAD_US exactly like a
 * satellite, otherwise it would play ahead of every other speaker. Must hold the
 * lead (~21 kB) with headroom; 32 kB is 181 ms. */
#define LOCAL_RING_BYTES (64 * 1024)

static const char *TAG = "stream";

static StreamBufferHandle_t local_ring;
static i2s_chan_handle_t i2s_tx;
static int sock = -1;
static volatile uint32_t sample_rate = 44100;
static uint32_t tx_rate = 44100;         /* what the DAC clock is actually set to */
static uint32_t rate_ema;                /* smoothed measured input rate */

static void retune_dac(uint32_t hz);

#if CONFIG_DANCEFLOOR_ENABLE_MARKER
static volatile int64_t s_marker_at;      /* local time we last pulsed */
#endif

/*
 * Ring position at which a tagged packet's audio begins, or -1 for none
 * pending. Playback pulses when it reaches this, so the pulse tracks the audio
 * through the buffer rather than being predicted from a clock.
 */
/*
 * 32-bit, not 64: read by the playback task while the receive task writes them,
 * and a 64-bit load is two instructions here -- a torn read gives a garbage
 * position and a wild marker. int32 holds 13 hours of frames at 44.1 kHz.
 */
static volatile int32_t s_marker_sample = -1;
static volatile int32_t s_samples_in;     /* frames written into local_ring */

/*
 * Phase tracking, matching the satellite.
 *
 * This unit publishes the timeline, so it should hold itself to it. Servoing on
 * buffer depth alone matches its playback RATE to its input but lets its
 * POSITION wander -- and since it wanders independently of every satellite, the
 * speakers separate even though each one's buffer looks perfectly stable.
 */
#define PHASE_Q_LEN 32
typedef struct {
    int32_t pos;
    int64_t play_at;
} phase_pt_t;

static phase_pt_t s_phase_q[PHASE_Q_LEN];
static volatile uint32_t s_phase_head, s_phase_tail;
static volatile int32_t s_phase_err_us;   /* + = playing late */
static volatile bool s_phase_valid;
/* Set when a splice steps the phase, so the shadow average in ring_monitor_task
 * forgets a history that describes the situation before it -- the satellite has
 * done this since "Forget the phase average after a splice". */
static volatile bool s_phase_stepped;
static volatile bool s_restart_pending;   /* flag the next packet */
/*
 * Set when local playback underruns. The play task then waits for local_start,
 * which is only assigned at a timeline start -- so without this the hub stays
 * silent until the stream stops and restarts, discarding every incoming byte in
 * the meantime (seen as fed-drop climbing to the full stream rate).
 *
 * Recovery restarts the timeline, which also re-anchors every satellite, so the
 * system comes back aligned rather than merely audible.
 */
static volatile bool s_underrun_recover;
static volatile int32_t s_restart_pos = -1;

/* Never splice more than this in one go -- a larger error means something a
 * splice will not fix, and 150 ms is audible even at a track change. */
#define MAX_SPLICE_MS 150

/* Held across a DAC retune, and the play task parks on it.
 * i2s_channel_write() returns immediately once the channel is disabled, so
 * without this the play task spins through the ring at memory speed -- a
 * measured 54 ms correction cost 177 ms of buffer. */
static volatile bool retuning;
static volatile int64_t local_start;   /* master-clock instant local playback begins */

void streamer_set_sample_rate(uint32_t hz)
{
    if (!hz) {
        return;
    }
    sample_rate = hz;
    /* Smooth it: single windows carry ~0.3% noise, and every retune glitches
     * audio. The servo below wants a stable baseline, not the latest sample. */
    rate_ema = rate_ema ? (rate_ema * 3 + hz) / 4 : hz;

    /*
     * Match the DAC clock to the measured input rate.
     *
     * Transmitting at 44100 while receiving 42400 drains the playback buffer at
     * ~7 kB/s -- the 250 ms of audio is gone in five seconds and never recovers.
     * That is a 4% mismatch: 40000 ppm, against the ~14 ppm crystal drift M6 is
     * designed for. No sample-level correction can absorb it; the clocks have to
     * agree.
     *
     * Matching is right whether the deficit is a genuinely slower source or lost
     * frames: either way this board only receives `hz` frames per second, so
     * playing them at `hz` is what keeps real time.
     *
     * 1% threshold: measurement noise is ~0.3%, and retuning glitches audio.
     */
    /* Big initial mismatch (44100 nominal vs ~42600 actual) is corrected once,
     * immediately. Everything finer is left to the servo, which uses the buffer
     * level rather than the noisy rate estimate. */
    if (i2s_tx && (hz > tx_rate + tx_rate / 100 || hz < tx_rate - tx_rate / 100)) {
        retune_dac(hz);
    } else {
        ESP_LOGI(TAG, "sample rate %" PRIu32 " Hz", hz);
    }
}

/* Bytes dropped because pcm_stream was full. Silent loss here looks exactly
 * like a starving ring, which is why it needs a counter. */
static volatile uint32_t s_feed_dropped;
static volatile uint32_t s_tx_fail;      /* sendto() rejections */

/*
 * Cumulative totals for a long run, never reset -- deliberately separate from
 * the per-window counters above, which are cleared every 5 s.
 *
 * A rate tells you what is happening now; a total tells you whether something
 * has been happening slowly for an hour. Nothing here had one, so the longest
 * evidenced session was seven minutes and a leak or a slow decay would have
 * been invisible. This project's own lesson: every real fault was invisible
 * until something counted it.
 */
static volatile uint32_t n_underruns;     /* local playback ran dry */
static volatile uint32_t n_restarts;      /* timeline restarted */
static volatile uint32_t n_splices;       /* track-boundary corrections applied */
static volatile uint32_t n_retunes;       /* DAC clock changes that succeeded */
static volatile uint32_t n_retunes_bad;   /* refused or failed */
static volatile uint32_t n_sta_left;      /* satellites disassociating */
static volatile uint32_t hw_play;         /* stack headroom, sampled in-task */
static volatile uint32_t hw_mon;

/*
 * Satellites are sent audio by UNICAST, not multicast.
 *
 * Group-addressed frames are never acknowledged and never retried, so any
 * corrupted frame is simply lost. Measured 20% loss across three different PHY
 * rates -- the rate was never the problem, the absence of retries was. Unicast
 * gets link-layer ACK and retransmission, which is what makes 802.11 reliable.
 *
 * The cost is that airtime scales with speaker count. At ~42 kB/s of SBC that is
 * affordable for a handful of units; it would not have been for 179 kB/s of PCM.
 *
 * Registration is implicit: satellites already send time probes every 250 ms, so
 * anything that has probed recently is listening.
 */
#define MAX_CLIENTS 8
#define CLIENT_TIMEOUT_US 10000000   /* forget a satellite that stops probing */

typedef struct {
    struct sockaddr_in addr;
    int64_t last_seen;
} client_t;

static client_t s_clients[MAX_CLIENTS];
static portMUX_TYPE s_clients_lock = portMUX_INITIALIZER_UNLOCKED;

static void client_seen(const struct sockaddr_in *from)
{
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_clients_lock);
    int free_slot = -1;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_clients[i].last_seen &&
            s_clients[i].addr.sin_addr.s_addr == from->sin_addr.s_addr) {
            s_clients[i].last_seen = now;
            portEXIT_CRITICAL(&s_clients_lock);
            return;
        }
        if (!s_clients[i].last_seen && free_slot < 0) {
            free_slot = i;
        }
    }
    if (free_slot >= 0) {
        s_clients[free_slot].addr = *from;
        s_clients[free_slot].last_seen = now;
    }
    portEXIT_CRITICAL(&s_clients_lock);
}

uint32_t streamer_take_dropped(void)
{
    uint32_t d = s_feed_dropped;
    s_feed_dropped = 0;
    return d;
}

void streamer_feed(const uint8_t *pcm, uint32_t len)
{
    /*
     * Straight to the local ring. There is no intermediate PCM buffer any more:
     * satellites receive SBC, so nothing needs PCM except this speaker.
     *
     * Deliberately not gated on local_start. sbc_in decodes and feeds a packet
     * before calling streamer_send_sbc(), which is what sets local_start -- so
     * gating discarded the first packet's audio here while the satellites got
     * it, leaving this unit permanently one packet (~20 ms) behind them. The
     * ring is reset at timeline start, so anything fed early is cleared anyway.
     */
    if (!local_ring) {
        return;
    }
    size_t sent = xStreamBufferSend(local_ring, pcm, len, 0);   /* must not block */
    if (sent < len) {
        s_feed_dropped += len - sent;
    }
    s_samples_in += (int32_t)(sent / (AUDIO_CHANNELS * sizeof(int16_t)));
}

/* Called as a tagged packet is about to be queued: its audio starts here. */
void streamer_mark_here(void)
{
    if (s_marker_sample < 0) {
        s_marker_sample = s_samples_in;
    }
}

/*
 * Capture where the next packet's audio will start, before it is fed.
 *
 * Needed because a packet's play_at is not known until streamer_send_sbc()
 * computes it, by which point s_samples_in has already advanced past that
 * packet's audio. The pair only means anything if both halves refer to the
 * same instant in the stream.
 */
static int32_t s_pending_pos;

void streamer_request_restart(void)
{
    s_restart_pending = true;
}

void streamer_send_meta(const uint8_t *meta, uint16_t len)
{
    if (sock < 0 || len > sizeof(((meta_msg_t *)0)->payload)) {
        return;
    }
    meta_msg_t msg = { .type = MSG_META };
    memcpy(msg.payload, meta, len);

    portENTER_CRITICAL(&s_clients_lock);
    client_t snapshot[MAX_CLIENTS];
    memcpy(snapshot, s_clients, sizeof(snapshot));
    portEXIT_CRITICAL(&s_clients_lock);

    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (snapshot[i].last_seen) {
            sendto(sock, &msg, sizeof(msg), 0,
                   (struct sockaddr *)&snapshot[i].addr, sizeof(snapshot[i].addr));
        }
    }
}

void streamer_begin_packet(void)
{
    s_pending_pos = s_samples_in;
}

void streamer_send_sbc(const uint8_t *sbc, uint16_t len, uint32_t frames, bool marker)
{
    static audio_msg_t msg;
    static uint32_t seq;
    static int64_t next_play_at;

    if (sock < 0 || len == 0 || len > AUDIO_MAX_PAYLOAD || frames == 0) {
        return;
    }

    int64_t now = esp_timer_get_time();
    int64_t target = now + LEAD_US;

    /*
     * Recovering from a local underrun. The flag cannot be raised here: the
     * timeline-start branch below clears s_restart_pending, because a start
     * throws away any track boundary that was waiting for the old timeline. So
     * the request to bring the satellites with us was set and then wiped in the
     * same call, and no satellite has ever received it.
     */
    bool recovered = false;
    if (s_underrun_recover) {
        s_underrun_recover = false;
        next_play_at = 0;              /* fall into the timeline-start path */
        recovered = true;
    }

    /* A timeline start invalidates the position this packet captured, because
     * the ring it referred to is about to be cleared. */
    bool started = false;

    if (next_play_at == 0) {
        next_play_at = target;
        local_start = target;              /* our own speaker joins the timeline */
        xStreamBufferReset(local_ring);
        s_samples_in = 0;                  /* same origin as the reset ring */
        s_pending_pos = 0;                 /* and so does anything flagged here */
        s_marker_sample = -1;
        s_phase_head = s_phase_tail = 0;
        s_phase_valid = false;
        s_restart_pos = -1;
        s_restart_pending = false;
        started = true;
        n_restarts++;
        ESP_LOGI(TAG, "timeline start");
    } else if (llabs(next_play_at - target) > RESYNC_US) {
        static int64_t last_warn;
        int64_t err = next_play_at - target;
        next_play_at = target;
        if (now - last_warn > 2000000) {
            last_warn = now;
            ESP_LOGW(TAG, "timeline off by %lld us, resyncing", err);
        }
    }

    msg.type = MSG_AUDIO;
    msg.format = AUDIO_FMT_SBC;
    msg.payload_len = len;
    msg.seq = seq++;
    msg.sample_rate = sample_rate;
    msg.frames = frames;
    msg.marker = marker ? 1 : 0;
    /*
     * Two different events wear the same flag on the wire, and they differ in
     * whether this unit splices too.
     *
     * A track boundary: every unit nulls its phase when playback reaches the
     * flagged audio, this one included. That is the case the flag was built for.
     *
     * A timeline restart after a local underrun: the satellites did NOT restart
     * with us. They are still playing against the old timeline, and the stamps
     * they are about to receive step by up to RESYNC_US -- inside the 150 ms a
     * splice can absorb, which is why telling them is worth doing at all. We
     * must not splice: our phase was just re-anchored to zero by construction,
     * while s_phase_err_us still holds whatever it read before the underrun.
     * Acting on that would cut up to MAX_SPLICE_MS out of the first audio of
     * the new timeline for no reason.
     */
    msg.restart = (s_restart_pending || recovered) ? 1 : 0;
    if (s_restart_pending) {
        s_restart_pending = false;
        if (s_restart_pos < 0) {
            s_restart_pos = s_pending_pos;   /* our own copy of the same boundary */
        }
        ESP_LOGW(TAG, "track boundary flagged at seq %" PRIu32, msg.seq);
    } else if (recovered) {
        ESP_LOGW(TAG, "timeline restart flagged at seq %" PRIu32
                      " -- satellites re-splice, we do not", msg.seq);
    }

    /*
     * Position captured before the audio was fed, paired with the time it is
     * due -- the playback task compares the two when it gets there.
     *
     * Skipped at a timeline start, and that matters more than it looks.
     * streamer_begin_packet() snapshots s_samples_in before this packet is
     * decoded and fed; the branch above then resets s_samples_in to zero and
     * clears the ring, so the snapshot describes an origin that no longer
     * exists. On the first start it is legitimately zero, but on an underrun
     * restart it is every frame fed since the last one -- hours of them.
     *
     * Queued, that entry sits at the head of a queue playback reaches only
     * after playing the same hours of audio. The tail never advances, the queue
     * fills, no further points are ever recorded, and s_phase_valid stays false
     * for good: the ring servo stops, anchor_due stays zero, and the visualiser
     * is fed a due of zero for ever -- hue frozen and no envelope decay, while
     * every satellite carries on. One local underrun took the hub's strip out
     * for the rest of the session.
     *
     * Nothing is lost by skipping it. This packet's audio went into the ring
     * before the reset cleared it, so it has no position to record; the next
     * packet lands at zero in the fresh ring and records itself correctly
     * against the timeline this call just advanced.
     */
    uint32_t nq = (s_phase_head + 1) % PHASE_Q_LEN;
    if (!started && nq != s_phase_tail) {
        s_phase_q[s_phase_head].pos = s_pending_pos;
        s_phase_q[s_phase_head].play_at = next_play_at;
        s_phase_head = nq;
    }
    msg.play_at = next_play_at;
    memcpy(msg.payload, sbc, len);

    size_t bytes = AUDIO_MSG_BYTES(len);

    portENTER_CRITICAL(&s_clients_lock);
    client_t snapshot[MAX_CLIENTS];
    memcpy(snapshot, s_clients, sizeof(snapshot));
    portEXIT_CRITICAL(&s_clients_lock);

    /*
     * Unicast to each registered listener. Multicast was removed entirely: it is
     * never acknowledged and never retried, which cost ~20% of packets at every
     * PHY rate tried (1, 6 and 24 Mbps all landed near the same loss). Unicast
     * gets link-layer ACK and retransmission and measured essentially clean.
     *
     * Airtime now scales with speaker count, which is affordable at ~42 kB/s of
     * SBC and would not have been at 179 kB/s of PCM.
     */
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!snapshot[i].last_seen) {
            continue;
        }
        if (now - snapshot[i].last_seen > CLIENT_TIMEOUT_US) {
            portENTER_CRITICAL(&s_clients_lock);
            s_clients[i].last_seen = 0;
            portEXIT_CRITICAL(&s_clients_lock);
            continue;
        }
        if (sendto(sock, &msg, bytes, 0,
                   (struct sockaddr *)&snapshot[i].addr, sizeof(snapshot[i].addr)) < 0) {
            s_tx_fail++;
        }
    }

    /* The timeline advances by the audio actually sent, not by wall clock --
     * stamping "now + lead" each time would fold task jitter into playback. */
    next_play_at += (int64_t)frames * 1000000LL / (int64_t)sample_rate;
}

/* ------------------------------------------------------------------- wifi */

/*
 * Only to count them. A satellite dropping off is invisible otherwise -- the
 * driver logs it, but nothing accumulates it, so a link that flaps once an hour
 * over an evening looks identical to one that never does. One reason=209
 * SA-Query disassociation has already been seen.
 */
static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        n_sta_left++;
    }
}

static void wifi_start_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                       WIFI_EVENT_AP_STADISCONNECTED,
                                                       wifi_event, NULL, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /*
     * Fields below are set explicitly rather than left at zero from the
     * initialiser: dtim_period has a documented range of 1-10 and zero is
     * invalid, and pmf_cfg left unset makes some clients unhappy during the
     * WPA2 handshake -- which surfaces as "incorrect password" rather than
     * anything that points at the real cause.
     */
    wifi_config_t wc = {0};
    strcpy((char *)wc.ap.ssid, AP_SSID);
    strcpy((char *)wc.ap.password, AP_PASS);
    wc.ap.ssid_len = strlen(AP_SSID);
    wc.ap.max_connection = 8;
    wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wc.ap.channel = CONFIG_DANCEFLOOR_WIFI_CHANNEL;
    wc.ap.dtim_period = 1;
    /* Matches ESP-IDF's own softAP example. Setting capable=true is a deviation
     * that some clients refuse, so leave it alone. */
    wc.ap.pmf_cfg.required = false;

#if CONFIG_DANCEFLOOR_AP_OPEN
    wc.ap.authmode = WIFI_AUTH_OPEN;
    wc.ap.password[0] = '\0';
    ESP_LOGW(TAG, "AP is OPEN (no password) -- diagnostic build");
#endif

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Power save would park the radio between beacons and add tens of ms to
     * the packets whose timing we depend on. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    /*
     * TX power is left at the driver default (full). It was once capped at
     * 13 dBm to stop this radio swamping the Bluetooth receiver when both shared
     * one chip; the two-chip split removed that need, and the cap did real harm
     * by denying rate adaptation the SNR margin it needs.
     *
     * The channel stays pinned: it costs nothing and helps Bluetooth's adaptive
     * frequency hopping route around us.
     */
#if CONFIG_DANCEFLOOR_WIFI_PHY_RATE_MBPS > 0
    /*
     * Pinning the PHY rate exists for a reason that no longer applies, and is
     * kept only as a diagnostic knob. Set the rate to 0 to leave it alone.
     *
     * It was mandatory while audio went out by multicast: group-addressed frames
     * fall back to the 1 Mbps basic rate, where a 1041-byte frame costs 8.3 ms
     * and 172 packets/s needs 1.43 s of airtime per second -- it physically
     * cannot fit, and half the audio was dropped in the queue.
     *
     * Unicast has no such fallback. It uses rate adaptation, which is what fixed
     * the 23% loss that a fixed 24 Mbps caused. Note esp_wifi_internal_set_fix_rate()
     * applies to ALL transmission on the interface, so any non-zero value here
     * now pins unicast and disables that adaptation.
     */
#if   CONFIG_DANCEFLOOR_WIFI_PHY_RATE_MBPS >= 24
    const wifi_phy_rate_t want = WIFI_PHY_RATE_24M;
#elif CONFIG_DANCEFLOOR_WIFI_PHY_RATE_MBPS >= 12
    const wifi_phy_rate_t want = WIFI_PHY_RATE_12M;
#else
    const wifi_phy_rate_t want = WIFI_PHY_RATE_6M;
#endif
    esp_err_t rerr = esp_wifi_internal_set_fix_rate(WIFI_IF_AP, true, want);
    if (rerr != ESP_OK) {
        ESP_LOGE(TAG, "could not fix PHY rate (%s); needs "
                      "CONFIG_ESP_WIFI_AMPDU_TX_ENABLED=n", esp_err_to_name(rerr));
    } else {
        ESP_LOGW(TAG, "PHY rate pinned to %d Mbps -- rate adaptation is OFF for "
                      "unicast too; set to 0 to restore it",
                 CONFIG_DANCEFLOOR_WIFI_PHY_RATE_MBPS);
    }
#endif
    ESP_LOGI(TAG, "SoftAP \"%s\" pass \"%s\" ch %d, radio at defaults",
             AP_SSID, AP_PASS, CONFIG_DANCEFLOOR_WIFI_CHANNEL);
}

static void socket_start(void)
{
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    assert(sock >= 0);

    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SYNC_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    assert(bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) == 0);
}

/* ------------------------------------------------------ sync measurement */

#if CONFIG_DANCEFLOOR_ENABLE_MARKER
static QueueHandle_t s_edge_q;            /* satellite edge timestamps */

/*
 * Last cross-unit measurement, for the per-track summary below.
 *
 * A track boundary nulls phase on every unit, so cross-unit error resets there
 * and grows until the next one. That makes the reading taken just BEFORE a
 * boundary the one worth keeping: it is how far apart the speakers had drifted
 * over a whole track, which is the number to compare sessions and builds on.
 * Any other sample depends on where in the track cycle it was taken, and
 * comparing two of those produced three confident wrong diagnoses in a row.
 */
static volatile int64_t s_sync_err_us;
static volatile int64_t s_sync_at;        /* 0 = never measured */

/* This unit's own last boundary correction, for satellites to be compared
 * against when they report theirs. */
static volatile int32_t s_hub_splice_us;
static volatile int64_t s_hub_splice_at;  /* 0 = no boundary yet */

static void IRAM_ATTR monitor_isr(void *arg)
{
    (void)arg;
    int64_t now = esp_timer_get_time();
    BaseType_t hp = pdFALSE;
    xQueueSendFromISR(s_edge_q, &now, &hp);
    if (hp) {
        portYIELD_FROM_ISR();
    }
}

/*
 * Reports how far a satellite's audio is from this unit's, by comparing when
 * each pulsed for the same master-clock instant. This is the end-to-end number
 * the whole design exists to deliver -- everything else (clock offset, buffer
 * level, packet loss) is a means to it.
 */
static void monitor_task(void *arg)
{
    (void)arg;
    int64_t edge;
    while (xQueueReceive(s_edge_q, &edge, portMAX_DELAY) == pdTRUE) {
        int64_t mine = s_marker_at;
        if (mine == 0) {
            continue;
        }
        int64_t err = edge - mine;
        /* Markers are 2 s apart; anything near that is a missed pulse rather
         * than a sync error, and reporting it as one would mislead. */
        if (err > 500000 || err < -500000) {
            continue;
        }
        /* Kept for the track-boundary summary, which wants the last reading
         * before the splice rather than a scroll of them. */
        s_sync_err_us = err;
        s_sync_at = esp_timer_get_time();

        ESP_LOGW(TAG, "AUDIO SYNC: satellite %+lld us (%s)", err,
                 err >= 0 ? "late" : "early");
    }
}

static void marker_start(void)
{
    gpio_config_t out = {
        .pin_bit_mask = 1ULL << CONFIG_DANCEFLOOR_MARKER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&out));

    gpio_config_t in = {
        .pin_bit_mask = 1ULL << CONFIG_DANCEFLOOR_MONITOR_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&in));

    s_edge_q = xQueueCreate(4, sizeof(int64_t));
    assert(s_edge_q);
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_DANCEFLOOR_MONITOR_GPIO, monitor_isr, NULL));
    xTaskCreate(monitor_task, "syncmon", 3072, NULL, 9, NULL);

    ESP_LOGI(TAG, "sync markers on GPIO %d, watching GPIO %d -- bench instrument, "
                  "nothing corrects on it",
             CONFIG_DANCEFLOOR_MARKER_GPIO, CONFIG_DANCEFLOOR_MONITOR_GPIO);
}
#endif  /* CONFIG_DANCEFLOOR_ENABLE_MARKER */

/* --------------------------------------------------- local delayed playback */

/* I2S_NUM_1 by history: port 0 used to be the slave receiver from the bridge.
 * That link is now UART, but there is no reason to move this. */
static void i2s_start(uint32_t rate)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(rate),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = CONFIG_DANCEFLOOR_I2S_BCK_PIN,
            .ws   = CONFIG_DANCEFLOOR_I2S_LRCK_PIN,
            .dout = CONFIG_DANCEFLOOR_I2S_DATA_PIN,
            .din  = I2S_GPIO_UNUSED,
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_tx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx));
    tx_rate = rate;
    /* Compare this against the satellite's line. The sync marker fires when a
     * chunk is written, not when it is heard, so unequal output buffering shows
     * up as a fixed offset unrelated to clock sync. */
    ESP_LOGW(TAG, "OUTPUT: I2S external DAC, buffer %d x %d frames = %d ms",
             chan_cfg.dma_desc_num, chan_cfg.dma_frame_num,
             (int)(chan_cfg.dma_desc_num * chan_cfg.dma_frame_num * 1000 / rate));
}

/*
 * Widest DRIFT correction the servo may ask for, in Hz. Deliberately not
 * applied inside retune_dac(): the initial match to the measured input rate is
 * a different thing and is legitimately several percent (44100 nominal against
 * ~42600 measured), so a bound tight enough to be useful here would refuse it.
 */
#define RATE_TRIM_MAX_HZ 100

/* What could conceivably be an audio sample rate at all. Anything outside this
 * is a broken calculation, whoever asked for it. */
#define RATE_SANE_MIN 8000
#define RATE_SANE_MAX 192000

/* What a retune costs -- see the note on the satellite's copy. The channel down
 * is measurable here; the discarded DMA buffer is not measurable anywhere in
 * software, because those frames were counted as played. */
static volatile int32_t s_retune_phase_before;
static volatile bool    s_retune_watch;
static volatile int64_t s_retune_outage_us;

static void retune_dac(uint32_t hz)
{
    /*
     * Nothing computed may panic the speaker. The satellite aborted on exactly
     * this path when a wrapped phase error asked for a 4.29 GHz sample rate:
     * ESP_ERROR_CHECK turned a bad number into a dead unit. A refused retune
     * costs sync, which is recoverable; an abort is not.
     */
    if (hz < RATE_SANE_MIN || hz > RATE_SANE_MAX) {
        n_retunes_bad++;
        ESP_LOGE(TAG, "refusing a %" PRIu32 " Hz retune -- not a sample rate", hz);
        return;
    }

    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(hz);
    retuning = true;
    /* Timed from here, so the 2 ms park counts: playback is stopped for it just
     * as surely as for the disable itself. */
    const int64_t down_at = esp_timer_get_time();
    vTaskDelay(pdMS_TO_TICKS(2));        /* let the play task notice and park */

    esp_err_t err = i2s_channel_disable(i2s_tx);
    if (err == ESP_OK) {
        err = i2s_channel_reconfig_std_clock(i2s_tx, &clk);
        /* Re-enable either way: leaving the channel down stalls playback
         * silently, which looks like a dead board rather than a failed trim. */
        const esp_err_t on = i2s_channel_enable(i2s_tx);
        if (err == ESP_OK) {
            err = on;
        }
    }
    retuning = false;

    if (err != ESP_OK) {
        n_retunes_bad++;
        ESP_LOGE(TAG, "retune to %" PRIu32 " Hz failed (%s), staying at %" PRIu32,
                 hz, esp_err_to_name(err), tx_rate);
        return;
    }
    s_retune_outage_us = esp_timer_get_time() - down_at;
    n_retunes++;
    ESP_LOGW(TAG, "DAC clock retuned %" PRIu32 " -> %" PRIu32 " Hz, channel down %lld us",
             tx_rate, hz, s_retune_outage_us);
    tx_rate = hz;

    s_retune_phase_before = s_phase_err_us;
    s_retune_watch = true;

#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
    /* The disable above discarded the DMA buffer: ~32 ms that the playback task
     * counted as played and fed to the visualiser, and that nobody heard. Left
     * alone, every label after this runs early by that much, on this unit only.
     * Covers the initial rate match too, which comes through here as well. */
    visualiser_realign();
#endif
}

/*
 * Same shape as the satellite's play task, minus the clock conversion: here
 * master time is local time. Holding the first sample until its scheduled
 * instant is what puts this speaker on the same timeline as the rest.
 */
static void local_play_task(void *arg)
{
    (void)arg;
    static uint8_t chunk[AUDIO_CHUNK_BYTES];

    while (1) {
        if (local_start == 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        int64_t wait = local_start - esp_timer_get_time();
        if (wait > 2000) {
            vTaskDelay(pdMS_TO_TICKS((wait - 2000) / 1000));
        }
        while (esp_timer_get_time() < local_start) {
            /* spin the last stretch */
        }
        /* On the hub local time IS master time, so this is directly comparable
         * with the satellite's figure. A difference here is a difference in the
         * audio each unit is playing, which no amount of clock accuracy fixes. */
        ESP_LOGI(TAG, "local playback started: scheduled %lld, actual %lld (%+lld us)",
                 local_start, esp_timer_get_time(), esp_timer_get_time() - local_start);
        /* samples_played counts from the first sample played, which is the
         * first sample fed after the ring was reset at timeline start. Both
         * counters therefore share an origin -- do NOT reset s_samples_in here,
         * it has legitimately been counting the audio buffered during the wait. */
        int32_t samples_played = 0;
        /* Last phase point seen, for labelling audio with the instant it is due.
         * Zero until the first point lands; the visualiser waits rather than
         * aligning to a guess. */
        int32_t anchor_pos = 0;
        int64_t anchor_due = 0;

        while (1) {
            if (retuning) {
                /* Do not pull from the ring while the channel is down -- writes
                 * would return instantly and drain it. */
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
            hw_play = uxTaskGetStackHighWaterMark(NULL);   /* only valid in-task */
            size_t got = xStreamBufferReceive(local_ring, chunk, sizeof(chunk), pdMS_TO_TICKS(500));
            if (got == 0) {
                n_underruns++;
                ESP_LOGW(TAG, "local underrun, restarting timeline");
                local_start = 0;
                s_underrun_recover = true;
                break;
            }
            if (got < sizeof(chunk)) {
                memset(chunk + got, 0, sizeof(chunk) - got);
            }
            /* Local time IS master time here, so this is a direct read of how
             * far playback has slipped from the published timeline. */
            while (s_phase_tail != s_phase_head && samples_played >= s_phase_q[s_phase_tail].pos) {
                /*
                 * Correct for WHERE the crossing was noticed versus where it
                 * happened, which is most of this unit's phase noise.
                 *
                 * samples_played advances by AUDIO_FRAMES per iteration -- 5.8
                 * ms at 44.1 kHz -- so by the time the loop sees it has passed
                 * `pos`, it passed it up to a chunk ago, by an amount that
                 * depends on where pos falls on the chunk grid and is therefore
                 * uncorrelated sample to sample. Reading the clock here dates
                 * the crossing at "when I noticed", and the difference is pure
                 * quantisation noise on the servo's only input.
                 *
                 * Measured before this: two reads of s_phase_err_us in adjacent
                 * log lines, a millisecond apart, differing by 15.7 ms. That
                 * noise made the hub's own retune bench unmeasurable (scatter
                 * 2.9x the effect), produced a false 23 ms alarm, and is the
                 * "hub absolute phase does not settle" wart in clock-sync.md.
                 *
                 * The overshoot is known exactly, so this is arithmetic rather
                 * than a filter: writes are paced by the DAC, so the instant
                 * samples_played was `pos` is `overshoot / rate` ago.
                 */
                int32_t overshoot = samples_played - s_phase_q[s_phase_tail].pos;
                /*
                 * Capped at one chunk, because beyond that the pacing
                 * assumption is false. A splice advances samples_played by up
                 * to MAX_SPLICE_MS in a single step and those frames were
                 * discarded rather than played over time, so any point the jump
                 * carried us past cannot be dated this way. Capping leaves
                 * those readings no worse than they were before this
                 * correction existed.
                 */
                if (overshoot > AUDIO_FRAMES) {
                    overshoot = AUDIO_FRAMES;
                }
                const int64_t crossed_at = esp_timer_get_time()
                                         - (int64_t)overshoot * 1000000 / sample_rate;
                s_phase_err_us = (int32_t)(crossed_at - s_phase_q[s_phase_tail].play_at);
                s_phase_valid = true;
                if (s_retune_watch) {
                    s_retune_watch = false;
                    ESP_LOGW(TAG, "RETUNE COST: phase %+ld -> %+ld us (net %+ld), "
                                  "channel was down %lld us",
                             (long)s_retune_phase_before, (long)s_phase_err_us,
                             (long)(s_phase_err_us - s_retune_phase_before),
                             s_retune_outage_us);
                }
                /* Keep the last point: interpolating from it labels each chunk
                 * with the instant it is DUE, which every unit agrees on because
                 * they all got the same play_at. The visualiser cuts its
                 * analysis blocks on that. */
                anchor_pos = s_phase_q[s_phase_tail].pos;
                anchor_due = s_phase_q[s_phase_tail].play_at;
                s_phase_tail = (s_phase_tail + 1) % PHASE_Q_LEN;
            }

            /* Track boundary: snap phase to zero instead of letting the servo
             * walk it off over ~45 s. Only inaudible here. */
            int32_t rp = s_restart_pos;
            if (rp >= 0 && samples_played >= rp) {
                s_restart_pos = -1;
                int32_t max_frames = (int32_t)sample_rate * MAX_SPLICE_MS / 1000;
                /*
                 * Nothing measured since the last re-anchor: s_phase_err_us is
                 * whatever it read against the previous timeline, and it is not
                 * cleared. Splicing on it would cut up to MAX_SPLICE_MS of real
                 * audio to correct an error that no longer exists. Drop the
                 * boundary instead -- the servo will take out anything genuine.
                 */
                int32_t adj = s_phase_valid
                    ? (int32_t)((int64_t)s_phase_err_us * sample_rate / 1000000) : 0;
                if (adj > max_frames)  adj = max_frames;
                if (adj < -max_frames) adj = -max_frames;
                int32_t applied = 0;      /* what the splice actually moved */

                if (adj > 0) {
                    /* Its own buffer, not chunk: chunk holds the audio read at
                     * the top of this pass and not yet written to the DAC, so
                     * discarding into it dropped that and played the tail of the
                     * skipped region instead. Counted correctly either way, so
                     * nothing drifted -- it just played 5.8 ms of the wrong
                     * audio at every boundary. Same fix on the satellite. */
                    static uint8_t discard[AUDIO_CHUNK_BYTES];
                    int32_t left = adj;
                    while (left > 0) {
                        size_t want = (size_t)(left > AUDIO_FRAMES ? AUDIO_FRAMES : left)
                                      * AUDIO_CHANNELS * sizeof(int16_t);
                        size_t g = xStreamBufferReceive(local_ring, discard, want, 0);
                        if (g == 0) break;
                        left -= g / (AUDIO_CHANNELS * sizeof(int16_t));
                    }
                    samples_played += (adj - left);
                    applied = adj - left;
                    ESP_LOGW(TAG, "track boundary: skipped %ld ms to null phase",
                             (long)(applied * 1000 / (int32_t)sample_rate));
                } else if (adj < 0) {
                    static const uint8_t quiet[AUDIO_CHUNK_BYTES] = {0};
                    int32_t left = -adj;
                    size_t w = 0;
                    while (left > 0) {
                        int32_t n = left > AUDIO_FRAMES ? AUDIO_FRAMES : left;
                        size_t bytes = (size_t)n * AUDIO_CHANNELS * sizeof(int16_t);
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
                        /* This silence is genuinely played, so the visualiser
                         * sees it like everything else that reaches the DAC --
                         * the satellite already fed its own through write_audio()
                         * and the two must not differ. No scheduled instant to
                         * label it with; the realign below supplies one. */
                        visualiser_feed(quiet, bytes, 0);
#endif
                        i2s_channel_write(i2s_tx, quiet, bytes, &w, portMAX_DELAY);
                        left -= n;
                    }
                    applied = adj;
                    ESP_LOGW(TAG, "track boundary: inserted %ld ms to null phase",
                             (long)(-applied * 1000 / (int32_t)sample_rate));
                }
                if (adj != 0) {
                    n_splices++;
                    s_phase_stepped = true;   /* the average before it is stale */
                }

                /*
                 * One line per track for how far apart the speakers had drifted
                 * by the end of it -- the figure to compare sessions and builds
                 * on, because it is taken at the same point of every track
                 * cycle rather than wherever a log window happened to fall.
                 *
                 * The satellite figure is the marker: a physical measurement of
                 * when a sample reached the output, so it sees things no
                 * software reading can. The hub's splice is how much of its own
                 * error it had accumulated. They answer different questions and
                 * both belong here.
                 */
                s_hub_splice_us = (int32_t)((int64_t)applied * 1000000 / sample_rate);
                s_hub_splice_at = esp_timer_get_time();

                if (s_sync_at) {
                    ESP_LOGW(TAG, "TRACK DIVERGENCE: satellite %+lld us "
                                  "(marker, %lld ms before this boundary) | "
                                  "hub spliced %+ld ms | hub phase %+ld us",
                             s_sync_err_us,
                             (s_hub_splice_at - s_sync_at) / 1000,
                             (long)(s_hub_splice_us / 1000),
                             (long)s_phase_err_us);
                } else {
                    /* No marker wire -- the normal deployed case, since it is a
                     * bench instrument. Satellites report over WiFi instead, and
                     * their line arrives within PROBE_PERIOD_MS of this one. */
                    ESP_LOGW(TAG, "TRACK BOUNDARY: hub spliced %+ld ms | "
                                  "hub phase %+ld us | no marker fitted",
                             (long)(s_hub_splice_us / 1000),
                             (long)s_phase_err_us);
                }
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
                if (adj != 0) {
                    /* Samples were added to or taken out of what the visualiser
                     * counts, so its block boundaries and its due_us stopped
                     * describing the timeline. Each unit splices by its own
                     * phase error, so leaving this out is what stepped the
                     * strips apart at every track change. */
                    visualiser_realign();
                }
#endif
            }

            int32_t mark = s_marker_sample;
            if (mark >= 0 && samples_played >= mark) {
#if CONFIG_DANCEFLOOR_ENABLE_MARKER
                /* 200 us of busy-wait in the playback path -- see the Kconfig
                 * help. Nothing corrects on what it measures. */
                gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 1);
                s_marker_at = esp_timer_get_time();
                esp_rom_delay_us(MARKER_PULSE_US);
                gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 0);
#endif
                s_marker_sample = -1;
            }
            samples_played += AUDIO_FRAMES;

#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
            /* Fed here, at the DAC, and not where the audio arrived: ~200 ms of
             * buffer separates the two, and lights driven from the arrival side
             * run that far ahead of this unit's own speaker.
             *
             * The label is when this chunk is DUE, not the clock now -- see
             * visualiser.h. On the hub local time is master time, so no offset. */
            int64_t due = anchor_due
                ? anchor_due + (int64_t)(samples_played - anchor_pos) * 1000000 / sample_rate
                : 0;
            visualiser_feed(chunk, sizeof(chunk), due);
#endif
            size_t written = 0;
            if (i2s_channel_write(i2s_tx, chunk, sizeof(chunk), &written,
                                  portMAX_DELAY) != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(2));
            }
        }
    }
}

/* ------------------------------------------------------- time probe server */

static void probe_task(void *arg)
{
    (void)arg;
    uint8_t buf[64];
    struct sockaddr_in from;

    while (1) {
        /* recvfrom writes the actual address length back, so this must be reset
         * every iteration rather than hoisted out of the loop. */
        socklen_t from_len = sizeof(from);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &from_len);
        int64_t t2 = esp_timer_get_time();          /* stamp on arrival */

        /*
         * A satellite reporting what it corrected at a track boundary. Both
         * units splice by their own error against the same published timeline,
         * so the DIFFERENCE between the two corrections is how far apart they
         * had drifted over that track -- the same question the marker GPIO
         * answers physically, over the WiFi that is there anyway.
         */
        if (n >= (int)sizeof(splice_msg_t) && buf[0] == MSG_SPLICE) {
            splice_msg_t s;
            memcpy(&s, buf, sizeof(s));
            client_seen(&from);
            const int64_t age = s_hub_splice_at ? (t2 - s_hub_splice_at) / 1000 : -1;
            const char *who = inet_ntoa(from.sin_addr);
            if (age >= 0 && age < 10000) {
                ESP_LOGW(TAG, "TRACK DIVERGENCE (wifi): %s spliced %+ld ms "
                              "(phase %+ld us), hub spliced %+ld ms -> %+ld ms apart",
                         who, (long)(s.applied_us / 1000), (long)s.phase_us,
                         (long)(s_hub_splice_us / 1000),
                         (long)((s.applied_us - s_hub_splice_us) / 1000));
            } else {
                /* No boundary of our own to compare against -- the hub's phase
                 * was invalid, or this arrived nowhere near one. */
                ESP_LOGW(TAG, "satellite %s spliced %+ld ms (phase %+ld us), "
                              "no hub boundary to compare",
                         who, (long)(s.applied_us / 1000), (long)s.phase_us);
            }
            continue;
        }

        if (n < (int)sizeof(time_msg_t) || buf[0] != MSG_TIME_REQ) {
            continue;
        }
        client_seen(&from);      /* probing implies listening */

        time_msg_t msg;
        memcpy(&msg, buf, sizeof(msg));
        msg.type = MSG_TIME_RSP;
        msg.t2 = t2;
        msg.t3 = esp_timer_get_time();              /* stamp immediately before send */
        sendto(sock, &msg, sizeof(msg), 0, (struct sockaddr *)&from, from_len);
    }
}

/*
 * Servo the DAC clock on the buffer level, not on the measured rate.
 *
 * Chasing the measured rate cannot work: it carries ~0.3% noise, and whatever
 * error is left integrates straight into this buffer until it overflows or
 * empties. The level itself IS that integral, so nulling it removes the
 * accumulated error rather than the instantaneous one. Correction is spread over
 * ~40 s, well below the ~1% pitch shift a listener would notice.
 */
static void ring_monitor_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));

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
            ESP_LOGW(TAG, "HEALTH: up %llu s | heap %" PRIu32 " (min %" PRIu32 ") | "
                          "stack play %" PRIu32 " mon %" PRIu32 " | underruns %" PRIu32
                          " restarts %" PRIu32 " splices %" PRIu32 " retunes %" PRIu32
                          " (%" PRIu32 " refused) | sta-left %" PRIu32,
                     (unsigned long long)(esp_timer_get_time() / 1000000),
                     esp_get_free_heap_size(), esp_get_minimum_free_heap_size(),
                     hw_play, hw_mon, n_underruns, n_restarts, n_splices,
                     n_retunes, n_retunes_bad, n_sta_left);
        }

        if (local_start == 0 || rate_ema == 0) {
            continue;
        }

#if CONFIG_DANCEFLOOR_RETUNE_BENCH_S > 0
        /* Bench: retune to the rate already set, so the RETUNE COST line below
         * reports the cost of retuning and nothing else. One unit at a time. */
        static int bench_left;
        if (--bench_left <= 0) {
            bench_left = (CONFIG_DANCEFLOOR_RETUNE_BENCH_S + 4) / 5;
            ESP_LOGW(TAG, "BENCH: forcing a same-rate retune at %" PRIu32 " Hz", tx_rate);
            retune_dac(tx_rate);
            continue;
        }
#endif

        /*
         * The servo input, matching the satellite: a 4-sample EMA of the phase,
         * forgotten after a splice because the average from before it describes
         * a situation that no longer exists.
         *
         * This unit used to act on the raw reading, and the raw reading is far
         * noisier than anyone had established. Measured here, two reads of
         * s_phase_err_us one millisecond apart:
         *
         *   local ring ... | phase +26786 us (smoothed +8996 us)
         *   servo: phase +11108 us (smoothed +8996), ...
         *
         * 15.7 ms of swing between consecutive samples, with the average
         * sitting still at +9 ms through it. That is the "hub absolute phase
         * does not settle" wart in docs/clock-sync.md, quantified: the servo was
         * substantially triggering on measurement noise. A shadow run put two
         * of six retunes at the deadband edge, both of which the average would
         * have held.
         *
         * Honest caveat: cross-unit audio measured 0.5 to 2.5 ms with the raw
         * input, which is already the best this project has recorded, so this
         * is expected to reduce pointless retunes rather than to move that
         * number. If it moves it the wrong way, revert this commit -- the raw
         * value is still computed below and still logged.
         */
        static int32_t s_err_ema;
        static bool    s_err_ema_valid;
        if (s_phase_stepped || !s_phase_valid) {
            s_phase_stepped = false;
            s_err_ema_valid = false;   /* history describes a different world */
        }
        s_err_ema = s_err_ema_valid ? (s_err_ema * 3 + s_phase_err_us) / 4
                                    : s_phase_err_us;
        s_err_ema_valid = true;

        size_t filled = LOCAL_RING_BYTES - xStreamBufferSpacesAvailable(local_ring);
        ESP_LOGI(TAG, "local ring %u bytes (%lu ms) | phase %+ld us (smoothed %+ld us) | "
                      "tx-fail %" PRIu32 "/5s",
                 (unsigned)filled,
                 (unsigned long)(filled * 1000 / (sample_rate * AUDIO_CHANNELS * 2)),
                 (long)s_phase_err_us, (long)s_err_ema, s_tx_fail);
        s_tx_fail = 0;

        const int32_t target = (int32_t)(LEAD_US / 1000) *
                               (int32_t)(rate_ema * AUDIO_CHANNELS * 2 / 1000);
        int32_t err_frames = ((int32_t)filled - target) / (AUDIO_CHANNELS * 2);
        int32_t depth_ms = err_frames * 1000 / (int32_t)rate_ema;

        if (!s_phase_valid) {
            continue;
        }
        /*
         * Phase drives the correction; buffer depth is only a guard against
         * running empty or overflowing, which phase control would not see
         * coming. Late means behind the timeline, so play faster.
         *
         * Spread over ~100 s: at 40 s the loop was still correcting after the
         * error had gone and overshot to +8 ms. Real drift is ~0.8 ms/minute,
         * far slower than the correction needs to be.
         */
        int32_t adj = (int32_t)((int64_t)s_err_ema * rate_ema / 100000000LL);
        /* The drift correction is small by nature -- real drift is ~14 ppm.
         * Anything larger is a bad phase reading, not a rate error. */
        if (adj >  RATE_TRIM_MAX_HZ) adj =  RATE_TRIM_MAX_HZ;
        if (adj < -RATE_TRIM_MAX_HZ) adj = -RATE_TRIM_MAX_HZ;
        if (depth_ms < -120) {
            adj = -20;
        } else if (depth_ms > 120) {
            adj = 20;
        }
        uint32_t desired = (uint32_t)((int32_t)rate_ema + adj);

        /*
         * Deadband in phase error, not in rate -- see PHASE_DEADBAND_US. The
         * old tx_rate/5000 was documented as ~8 ms and is really ~20 ms. This
         * unit has always parked its playback across a retune, so its retunes
         * were never the expensive kind; the satellite's were, until it got the
         * same guard.
         */
        int32_t deadband = (int32_t)((int64_t)PHASE_DEADBAND_US * rate_ema / 100000000LL);
        if (deadband < 1) {
            deadband = 1;
        }

        /* Wait for the buffer to respond before correcting again -- the hub
         * had no cooldown at all, so it retuned every window and chased its own
         * previous correction. */
        /* The raw input is the shadow now. Kept so the comparison survives the
         * change, and so a revert has something to check itself against. */
        int32_t adj_raw = (int32_t)((int64_t)s_phase_err_us * rate_ema / 100000000LL);
        if (adj_raw >  RATE_TRIM_MAX_HZ) adj_raw =  RATE_TRIM_MAX_HZ;
        if (adj_raw < -RATE_TRIM_MAX_HZ) adj_raw = -RATE_TRIM_MAX_HZ;
        const uint32_t desired_raw = (uint32_t)((int32_t)rate_ema + adj_raw);

        static int cooldown;
        if (cooldown > 0) {
            cooldown--;
        } else {
            const bool ema_would = desired     > tx_rate + (uint32_t)deadband ||
                                   desired     < tx_rate - (uint32_t)deadband;
            const bool raw_would = desired_raw > tx_rate + (uint32_t)deadband ||
                                   desired_raw < tx_rate - (uint32_t)deadband;
            /*
             * Still logged, with the roles swapped: each of these is now a
             * retune the raw input would have made and the average declined, or
             * the reverse. If these become common AND the cross-unit figure
             * degrades, this commit is the thing to revert.
             */
            if (raw_would != ema_would) {
                ESP_LOGW(TAG, "SERVO DIVERGES: smoothed %+ld us -> %" PRIu32 " Hz (%s), "
                              "raw %+ld us -> %" PRIu32 " Hz (%s)",
                         (long)s_err_ema,      desired,     ema_would ? "retune" : "hold",
                         (long)s_phase_err_us, desired_raw, raw_would ? "retune" : "hold");
            }
            if (ema_would) {
                ESP_LOGI(TAG, "servo: smoothed %+ld us (raw %+ld), buffer %+ld ms "
                              "-> DAC %" PRIu32 " Hz",
                         (long)s_err_ema, (long)s_phase_err_us, (long)depth_ms, desired);
                retune_dac(desired);
                cooldown = 4;          /* ~20 s against a 100 s correction */
            }
        }
    }
}

void streamer_start(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    local_ring = xStreamBufferCreate(LOCAL_RING_BYTES, AUDIO_CHUNK_BYTES);
    assert(local_ring);

    wifi_start_ap();
    socket_start();
    i2s_start(sample_rate);
#if CONFIG_DANCEFLOOR_ENABLE_MARKER
    marker_start();
#endif

    ESP_LOGI(TAG, "free heap after WiFi init: %" PRIu32 " bytes", esp_get_free_heap_size());

    xTaskCreate(probe_task, "probe", 4096, NULL, 6, NULL);
    xTaskCreatePinnedToCore(local_play_task, "play", 4096, NULL, 8, NULL, 1);
    xTaskCreate(ring_monitor_task, "ringmon", 3072, NULL, 3, NULL);
    ESP_LOGI(TAG, "streaming on port %d, unicast to registered listeners", SYNC_PORT);
}
