/*
 * Dancefloor satellite -- M5.
 *
 * Joins the master's SoftAP, keeps its clock aligned with the master's, receives
 * unicast SBC packets, decodes them, and plays each at the instant it was
 * stamped for.
 *
 * No Bluetooth here: the master owns the phone connection. This board only
 * listens on WiFi and drives a DAC.
 *
 * M5 aligns the *start* of playback. Once I2S is running it free-runs on this
 * board's own crystal, so the two units drift apart at ~14 ppm -- that is M6's
 * problem, and the buffer-level log below is the signal it will act on.
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
#if CONFIG_DANCEFLOOR_USE_INTERNAL_DAC
#include "driver/dac_continuous.h"
#endif
#include "esp_event.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_app_desc.h"
#include "nvs_flash.h"

#include "sync_proto.h"
#include "sbc_link.h"
#include "sbc_decoder.h"
#include "visualiser.h"
#include "wifi_log.h"

#define AP_SSID    "dancefloor"
#define AP_PASS    "dancefloor"
#define MASTER_IP  "192.168.4.1"        /* esp_netif SoftAP default */

#define PROBE_PERIOD_MS 250             /* see docs/clock-sync.md §3 */

/*
 * Must hold the master's lead time plus headroom for jitter -- specifically
 * LEAD_US + RESYNC_US, which is how far the hub's timeline can legitimately be
 * from real time when a chunk is stamped.
 *
 * 80 kB, up from 64: 464 ms at 44.1 kHz stereo against 371 ms.
 *
 * The 16 kB buys the hub's RESYNC_US the headroom its own comment says it
 * wants and cannot have. Delivery from the Bluetooth bridge is bursty by
 * construction -- A2DP packets arrive ~23/s carrying ~43 ms each -- and the
 * measured swing of the hub's timeline against real time reaches +-132 ms
 * against a 120 ms threshold. So it trips about seven times a minute on
 * entirely normal delivery. Raising the threshold past the swing was blocked by
 * this constant: 200 + 120 = 320 already sat close enough to 371 that 150 was
 * not affordable.
 *
 * It is affordable here rather than on the hub because the hub is not the unit
 * that has to hold it. This one is, and it is the classic ESP32 -- 117 kB free
 * with a largest block of 106 kB, so 80 kB fits and 107 kB (which is what a
 * 500 ms lead would need) would not allocate at all. That asymmetry is worth
 * knowing before anyone proposes a longer lead on the strength of the S3 hub's
 * PSRAM: the buffer a lead has to fit in is on the other board.
 */
#define RING_BYTES  (80 * 1024)

static const char *TAG = "sat";

static int sock = -1;
static sync_est_t est;
static StreamBufferHandle_t ring;
#if CONFIG_DANCEFLOOR_USE_INTERNAL_DAC
static dac_continuous_handle_t dac_tx;
#else
static i2s_chan_handle_t i2s_tx;
#endif

/* Local-clock instant the next byte entering the ring should reach the DAC.
 * Zero means playback has not started. */
static volatile int64_t stream_start_local;
/*
 * When the current stream was anchored. Written by the receive task, read by
 * the drift servo, which holds its depth safety net off for a while after --
 * see DEPTH_NET_HOLD_US.
 */
static volatile int64_t anchor_at;
static volatile uint32_t stream_rate = 44100;
static uint32_t tx_rate = 44100;      /* what the output clock is actually set to */
/*
 * Local -> master conversion used by playback. Seeded at anchoring and then
 * slewed towards the live estimate -- see track_offset(). Owned by the playback
 * task once a stream is running; the receive task only writes it while playback
 * is parked waiting for one.
 */
static int64_t stream_offset;
static int64_t offset_slew_last;      /* when the slew last moved it */
/*
 * 32-bit, not 64: these are read by the playback task while the receive task
 * writes them, and a 64-bit load is two instructions on this CPU -- a torn read
 * yields a garbage position and a wild marker. int32 holds 13 hours of frames
 * at 44.1 kHz, which is longer than any party.
 */
static volatile int32_t marker_sample = -1;   /* ring position of a tagged packet */
static volatile int32_t samples_in;           /* frames written into the ring */

/*
 * Phase tracking.
 *
 * Servoing on buffer depth matches the playback RATE to the arrival rate, but
 * says nothing about POSITION. Depth also moves with network jitter, so the
 * servo nudges the rate in response to noise -- and two units seeing different
 * jitter end up with rates differing by ~0.03% at any instant, which is
 * several ms of relative movement between markers. Observed as 10-25 ms of
 * wander between hub and satellite, with each unit's own buffer perfectly
 * stable.
 *
 * Every packet says exactly when its first sample should play. Recording that
 * against the ring position it lands at gives a direct phase measurement when
 * playback reaches it: where we are, versus where the timeline says we should
 * be. Correcting that holds position rather than merely matching rates.
 *
 * Single producer (receive task), single consumer (playback), 32-bit indices,
 * so no locking is needed.
 */
#define PHASE_Q_LEN 32
typedef struct {
    int32_t pos;        /* ring frame position where this packet's audio starts */
    int64_t play_at;    /* master-clock instant that sample should be heard */
} phase_pt_t;

static phase_pt_t phase_q[PHASE_Q_LEN];
static volatile uint32_t phase_head, phase_tail;
static volatile int32_t phase_err_us;         /* + = playing late */
static volatile bool phase_valid;
/*
 * The last few raw readings, for the splice alone. Play task only -- pushed in
 * the crossing loop, read and reset in the splice, reset at the top of the
 * outer loop -- so no volatile and no lock, unlike everything around it.
 *
 * The servo has smoothed its input since it was measured triggering on noise;
 * the splice never did, and it is the larger correction of the two. See
 * sync_phase_hist_t. Nothing acts on this yet: it is measured against the raw
 * value first, on the same boundaries, and only then does the splice move.
 */
static sync_phase_hist_t phase_hist;
static volatile int32_t restart_pos = -1;     /* ring position of a track boundary */
/*
 * Set after a splice. The phase genuinely steps at that instant, so the running
 * average from before it describes a situation that no longer exists -- seen as
 * "phase -2153 us (smoothed +26992 us)", with the servo acting on the stale
 * figure. Splices are rare, so re-seeding here costs nothing; re-seeding on
 * every correction, as an earlier version did, destroys the smoothing entirely.
 */
static volatile bool phase_stepped;

/* Never splice more than this in one go. A larger error means something is
 * wrong that a splice will not fix, and a 150 ms jump is very audible even at a
 * track change. */
#define MAX_SPLICE_MS 150

/*
 * Beyond this, the phase reading is not describing our playback at all.
 *
 * Drift is ~0.8 ms per minute and delivery jitter is a few ms, so a whole
 * second cannot be either. What it does mean is that the stamps arriving now
 * were issued against a different clock origin than the one playback anchored
 * to. Servoing on that is meaningless; re-anchoring is the only thing that
 * helps.
 */
#define PHASE_INSANE_US 1000000

/*
 * What an anchorable packet looks like. See the refusals in handle_audio().
 *
 * The hub stamps every chunk LEAD_US = 200 ms ahead, so a healthy packet
 * arrives with most of that still in front of it -- a good anchor was measured
 * at "in 154 ms". This is half of that lead, and the number is not arbitrary
 * caution: the scheduled wait below is the ONLY thing that prefills the ring,
 * so whatever lead survives to here is the prefill, and half the design depth
 * is the least worth starting on.
 *
 * It was 20 ms, chosen as "the floor below which prefill is not worth having",
 * and that was the wrong test. A run cleared it by four milliseconds: 144
 * packets refused, then one accepted at +24 ms, anchoring with `buffer 0 ms`.
 * The ring then overfilled to 400 ms behind it (ring-full 59), phase reached
 * +118 ms, and the servo spent over 200 seconds walking it back. The guard
 * fired 144 times and still let through the one that mattered.
 *
 * There is a real tension in the value, and it is not resolved so much as
 * chosen. The hub's RESYNC_US allows its timeline to wander 150 ms from real
 * time before slewing, so a perfectly healthy packet arriving during a trough
 * can show as little as ~50 ms of lead -- below this floor. Such a packet WILL
 * be refused. That is deliberate: anchoring mid-trough is how a stream starts
 * with a lead it cannot keep, and troughs recover within a second or two, so
 * refusing costs a second and buys an anchor taken on the recovery instead.
 * ANCHOR_GIVE_UP_US bounds the cost if the trough is not a trough.
 *
 * Tied by convention to the hub's LEAD_US, which this unit cannot see. If the
 * lead ever changes, this is the second place to look.
 *
 * A second between anchors, against a hub that would supply forty-nine packets
 * in that time: if none of them anchors, the link is not in a state a re-anchor
 * can fix.
 *
 * Five seconds before giving up and anchoring anyway. Long enough that no
 * plausible burst of lateness reaches it, short enough that a genuine
 * lead/path mismatch does not leave a speaker silent for a whole track.
 */
#define ANCHOR_MIN_LEAD_US     100000
#define ANCHOR_MIN_INTERVAL_US 1000000
#define ANCHOR_GIVE_UP_US      5000000

/*
 * A gap beyond this is an outage, not jitter, and is re-anchored rather than
 * filled with silence. See the reasoning at the fill in handle_audio().
 *
 * 150 ms is about seven packets. Normal loss on a healthy link is one to three
 * -- 20 to 60 ms -- so this sits well clear of anything that should be filled,
 * while staying below the 200 ms RING_TARGET_MS that a fill this size would
 * otherwise push the ring past.
 */
#define GAP_RESYNC_MS 150

/* How long after an anchor the drift servo ignores buffer depth and servos on
 * phase alone. See the safety net in drift_task() for what it was doing to a
 * ring that had simply not finished filling yet. */
#define DEPTH_NET_HOLD_US      20000000

/*
 * A track-boundary correction waiting to be reported to the hub, so it can
 * print how far apart the units had drifted -- see splice_msg_t. Written by
 * playback, sent by the probe task, because a sendto() in the audio path is
 * exactly the kind of thing that costs a buffer.
 */
static volatile int32_t splice_report_us;
static volatile int32_t splice_report_phase;
/* SHADOW: the correction the median would have produced instead. Acted on by
 * nothing here; the hub prints it beside the real one so both units' figures
 * are compared at the same boundary. See splice_msg_t.applied_med_us. */
static volatile int32_t splice_report_med;
static volatile bool    splice_report_pending;

/*
 * The TSF-derived clock offset, and when it was last updated.
 *
 * Written by the receive task, read by playback. Zero `at` means no usable TSF
 * message has arrived, in which case everything falls back to the probe
 * estimator exactly as before -- both units may have TSF unavailable, the
 * satellite may not have associated yet, or the hub may be an older build that
 * does not send MSG_TSF at all.
 */
static volatile int64_t tsf_offset_us;
static volatile int64_t tsf_offset_at;
static volatile uint32_t n_tsf_used;      /* anchors that used TSF */
static volatile uint32_t n_tsf_fallback;  /* anchors that fell back */

/*
 * How stale a TSF offset may be and still be preferred over the estimator.
 * Messages arrive with every probe reply, 4/s, so a second means several have
 * been missed and the link is not healthy enough to trust the last one.
 */
#define TSF_MAX_AGE_US 1000000

/*
 * Cumulative totals for a long run, never reset. The 5 s lines answer "what is
 * happening now"; only a total answers "has this been happening slowly for an
 * hour", and nothing here had one. See the hub's copy.
 */
static volatile uint32_t n_underruns;     /* playback ran dry */
static volatile uint32_t n_reanchors;     /* streams anchored, first included */
static volatile uint32_t n_splices;       /* track-boundary corrections applied */
static volatile uint32_t n_retunes;
static volatile uint32_t n_retunes_bad;
static volatile uint32_t n_gaps;          /* lost-packet gaps filled with silence */
static volatile uint32_t n_wifi_drops;    /* disconnects from the hub's AP */
/*
 * The receive path's own instruments, counted here rather than logged there.
 *
 * These three used to be an ESP_LOGW each, per event, from inside
 * handle_audio() -- which runs in rx_task, which is the only thing draining a
 * UDP mailbox six datagrams deep against ~136 datagrams a second. The console
 * is a 115200-baud UART, so a ~60-character line is ~5 ms of blocking write.
 *
 * That closes a loop: packet loss makes lines, lines block the receive task,
 * a blocked receive task overflows the mailbox, and the overflow is more loss.
 * A run of it printed several hundred lines across six seconds and the loss
 * outlived the disturbance that started it by about that much.
 *
 * So the audio path increments and drift_task talks, within 5 s, from a task
 * that can afford to wait on a UART. Cumulative, like every other counter here;
 * the narration below prints the window by subtracting what it said last time.
 */
static volatile uint32_t n_gap_frames;    /* silence inserted for lost packets */
static volatile uint32_t n_gap_short;     /* gap fills the ring could not take */
static volatile uint32_t n_gap_short_frames;
static volatile uint32_t n_ring_full;     /* decoded blocks dropped, ring full */
static volatile uint32_t n_gap_resyncs;   /* gaps too large to fill, re-anchored */
static volatile uint32_t n_anchor_upgrades; /* provisional anchors replaced */
/*
 * Set by the receive task when a gap is too large to fill, cleared by the
 * playback task when it parks. See GAP_RESYNC_MS.
 */
static volatile bool resync_request;
/*
 * Set when ANCHOR_GIVE_UP_US forced an anchor onto a packet that was already
 * late. Playback is running but its position is known to be wrong, so the
 * receive path keeps watching for a packet it could have anchored on properly.
 */
static volatile bool anchor_provisional;
static volatile uint32_t n_anchor_late;   /* anchors refused, play_at already past */
static volatile uint32_t n_anchor_soon;   /* anchors refused, one just happened */
/*
 * Phase points dropped because phase_q was full. The only loss path in this
 * file that had no counter, which is the one thing the rest of this system is
 * built not to allow: every real fault here was invisible until something
 * counted it. A full queue means playback is not consuming points as fast as
 * the receive path records them, and the servo silently stops getting fresh
 * input while every log line still reads normally.
 */
static volatile uint32_t n_phase_drop;
/*
 * Ring reads that came back short of a full chunk, and the frames of silence
 * padded in to cover them.
 *
 * Suspected, not established, which is why this is a counter and not a fix. The
 * pad is played but was never in the ring, while samples_played advances by a
 * whole chunk regardless -- so if it happens, every later phase point is
 * displaced by the pad and the servo's only input carries a permanent bias.
 * That is the exact shape of the "silence inserted for a lost packet was not
 * counted in samples_in" bug, which put this unit ~20 ms out per loss and
 * stayed hidden because the marker was derived from the same count.
 *
 * The ring's trigger level is one chunk, so a short read means the 500 ms
 * timeout expired on a partly-filled ring -- a near-underrun. If these stay
 * zero over a long session the concern is latent and the fix can ride along
 * with anything; if they do not, n_short_frames IS the bias, in frames.
 */
static volatile uint32_t n_short_reads;
static volatile uint32_t n_short_frames;
/*
 * TSF samples whose read pair took longer than TSF_SPAN_MAX_US -- i.e. samples
 * something preempted between the two counter reads, so the offset they carry
 * is off by whatever landed in the gap.
 *
 * COUNTED, NOT ENFORCED. TSF is the anchor clock source now, and a threshold
 * chosen blind could silently demote it to the probe estimator, which is worse
 * -- that is a regression wearing no log line. This says what the reject rate
 * WOULD be, so the threshold can be chosen from the distribution instead.
 */
#define TSF_SPAN_MAX_US 100
static volatile uint32_t n_tsf_wide;
/*
 * When this unit went off the air, and when it came back.
 *
 * The suspicion being measured: nothing invalidates the probe estimator's
 * window on a disconnect. sync_est_offset() selects the lowest-RTT sample in a
 * 10-sample window and neither it nor sync_est_settled() decays with time, so
 * after an outage of any length the unit may anchor on an offset measured
 * before the drop -- and an offset error at the anchor is baked in for the life
 * of the stream, since play_at is consulted once. The first anchor after a
 * rejoin therefore says which clock it used and how stale the estimator's
 * newest sample was. If it reads "TSF" the concern does not arise, because TSF
 * is re-derived from a fresh beacon; if it reads "probe" with an age spanning
 * the outage, it does.
 */
static volatile int64_t wifi_down_at;
static volatile int64_t rejoined_at;      /* 0 = the next anchor is not the first */
static volatile int64_t est_newest_at;    /* when the newest probe landed */
static volatile uint32_t n_frames_rx;     /* analysis frames taken from the hub */
static volatile uint32_t n_frames_bad;    /* ... and rejected, wrong size */
static volatile uint32_t hw_play;         /* stack headroom, sampled in-task */
static volatile uint32_t hw_drift;

/*
 * Heap, dated, and allocation failures made audible. The hub's copy carries the
 * reasoning; this is the same instrument on the other unit.
 *
 * It is here despite no pressure ever having been observed on a satellite --
 * 52 kB free analysing locally, 118 kB being given its frames, against a hub
 * that reached 2040 bytes. Which is the point: the value of a windowed minimum
 * is that it says nothing, every minute, until the minute it does. A counter
 * that only exists on the unit already known to be sick cannot tell you the
 * other one just got sick too, and these two units do not have the same job or
 * the same failure.
 */
static volatile uint32_t heap_min_window = UINT32_MAX;
static volatile uint32_t n_alloc_fail;
static volatile uint32_t alloc_fail_size;   /* the largest request that failed */
static volatile uint32_t alloc_fail_caps;

/*
 * Records only, and in IRAM. IDF marks heap_caps_alloc_failed() HEAP_IRAM_ATTR
 * because the heap is usable with the flash cache disabled, so a hook in flash
 * would fault when reached from an ISR or during a flash write -- a diagnostic
 * for running out of memory that crashes under the one condition it exists to
 * observe. It also runs inside the allocator, and ESP_LOGx allocates, so
 * drift_task does the talking within 5 s.
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

/* Target buffer depth: the hub stamps audio ~200 ms ahead, so in steady state
 * that much should be sitting here waiting. */
#define RING_TARGET_MS 200

/*
 * TEMPORARY: does i2s_channel_disable() discard the DMA descriptors or drain
 * them? The depth is already on the OUTPUT line at boot; only this is unknown,
 * and the two answers predict opposite signs for the phase step a retune
 * causes. A discard leaves the descriptors empty, so writes return without
 * blocking until they refill; a drain leaves nothing to refill. Count the
 * frames that go in before the first write that blocks.
 *
 * Play task only -- it arms and clears these, so no volatile. Delete once the
 * question is settled.
 */
#define REFILL_FAST_US 1000     /* below this, the write did not block */
static bool    s_refill_active;
static int32_t s_refill_frames;
/* Which emptying this refill follows. A start and a retune drain the DMA for
 * different reasons and the startup one happens on every unit at once, so a
 * line that cannot tell them apart cannot answer the question either. */
static const char *s_refill_why = "start";

/* ------------------------------------------------------------------- wifi */

/*
 * Without this a failed association is completely silent: esp_wifi_connect() is
 * called once, and if it does not succeed nothing logs it and nothing retries.
 * A satellite that quietly never joins is far worse than one that says so.
 */
static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *d = data;
        n_wifi_drops++;
        /* Only the first drop of a streak dates the outage: repeated
         * association failures raise this repeatedly, and taking the latest
         * would measure the last retry rather than how long the unit was off
         * the air, which is the number that matters to playback. */
        if (wifi_down_at == 0) {
            wifi_down_at = esp_timer_get_time();
        }
        ESP_LOGW(TAG, "disconnected from \"%s\" (reason %d), retrying",
                 AP_SSID, d->reason);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *e = data;
        if (wifi_down_at) {
            /*
             * How long this unit was actually off the air. Playback survives
             * roughly the ring's worth of it and then underruns, so this is the
             * number that says whether a drop cost a glitch or a re-anchor --
             * and it is the baseline any change to the reconnect path has to
             * beat. rejoined_at arms the report on the next anchor.
             */
            ESP_LOGW(TAG, "rejoined \"%s\" after %lld ms",
                     AP_SSID, (esp_timer_get_time() - wifi_down_at) / 1000);
            wifi_down_at = 0;
            rejoined_at = esp_timer_get_time();
        }
        ESP_LOGI(TAG, "joined \"%s\", IP " IPSTR, AP_SSID, IP2STR(&e->ip_info.ip));
    }
}

static void wifi_start_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wc = {0};
    strcpy((char *)wc.sta.ssid, AP_SSID);
#if CONFIG_DANCEFLOOR_AP_OPEN
    wc.sta.password[0] = '\0';
    wc.sta.threshold.authmode = WIFI_AUTH_OPEN;
#else
    strcpy((char *)wc.sta.password, AP_PASS);
#endif
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                       wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                       wifi_event, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));

#if CONFIG_DANCEFLOOR_DISABLE_PMF
    /*
     * Both ends, so neither advertises the capability -- see the hub's copy for
     * what PMF was doing to this link. This unit is the one that was failing to
     * answer the SA Query and being thrown off for it, and it is also the one
     * that never noticed: the hub counted two disassociations while this
     * counted zero.
     *
     * Must sit between esp_wifi_set_config() and esp_wifi_start().
     */
    const esp_err_t pmf = esp_wifi_disable_pmf_config(WIFI_IF_STA);
    ESP_LOGW(TAG, "PMF disabled on the station: %s", esp_err_to_name(pmf));
#endif

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    /* STA_START triggers the first connect; disconnects retry from the handler. */
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
    /* No multicast group to join: audio arrives by unicast, and the time probes
     * this unit already sends are what register it with the hub. */
}

/* -------------------------------------------------------------------- i2s */

#if CONFIG_DANCEFLOOR_USE_INTERNAL_DAC
static void i2s_start(uint32_t rate)
{
    /* ALTER mode walks the DMA buffer across both channels in turn, so a stream
     * of interleaved left/right bytes comes out as stereo on GPIO 25 and 26. */
    dac_continuous_config_t cfg = {
        .chan_mask = DAC_CHANNEL_MASK_ALL,
        .desc_num = 8,
        .buf_size = 2048,
        .freq_hz = rate,
        .offset = 0,
        .clk_src = DAC_DIGI_CLK_SRC_DEFAULT,
        .chan_mode = DAC_CHANNEL_MODE_ALTER,
    };
    ESP_ERROR_CHECK(dac_continuous_new_channels(&cfg, &dac_tx));
    ESP_ERROR_CHECK(dac_continuous_enable(dac_tx));
    /* Depth matters for the sync marker: the pulse fires when a chunk is
     * written, so any difference in output buffering between units appears as a
     * fixed offset that has nothing to do with clock sync. */
    ESP_LOGW(TAG, "OUTPUT: internal DAC (8-bit), GPIO 25=L 26=R, "
                  "buffer 8 x 2048 B = %d ms",
             (8 * 2048) * 1000 / (int)(rate * 2));
}

/* int16 signed interleaved -> uint8 unsigned, which is what the DAC wants. */
static void dac_write(const uint8_t *pcm, size_t bytes)
{
    static uint8_t u8[1024];
    const int16_t *src = (const int16_t *)pcm;
    size_t samples = bytes / sizeof(int16_t);

    while (samples) {
        size_t n = samples > sizeof(u8) ? sizeof(u8) : samples;
        for (size_t i = 0; i < n; i++) {
            u8[i] = (uint8_t)((src[i] >> 8) + 128);
        }
        size_t loaded = 0;
        dac_continuous_write(dac_tx, u8, n, &loaded, -1);
        src += n;
        samples -= n;
    }
}
#else
static void i2s_start(uint32_t rate)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    /* One descriptor per chunk, so a write never spans two and the disable
     * waits at most one descriptor period for it. See the hub's copy for the
     * mechanism; both units must carry it, because this also sets the output
     * pipeline latency the servo absorbs at startup. */
    chan_cfg.dma_frame_num = AUDIO_FRAMES;
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
    ESP_LOGW(TAG, "OUTPUT: I2S external DAC, buffer %d x %d frames = %d ms",
             chan_cfg.dma_desc_num, chan_cfg.dma_frame_num,
             (int)(chan_cfg.dma_desc_num * chan_cfg.dma_frame_num * 1000 / rate));
}

static void dac_write(const uint8_t *pcm, size_t bytes)
{
    size_t written = 0;
    const int64_t w0 = s_refill_active ? esp_timer_get_time() : 0;
    /* Second line of defence behind the `retuning` park: a failed write does
     * NOT block, so ignoring it lets this task spin through the ring at memory
     * speed. That is what a retune used to cost. */
    if (i2s_channel_write(i2s_tx, pcm, bytes, &written, portMAX_DELAY) != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (s_refill_active) {
        if (esp_timer_get_time() - w0 < REFILL_FAST_US) {
            s_refill_frames += (int32_t)(written / (AUDIO_CHANNELS * sizeof(int16_t)));
        } else {
            s_refill_active = false;
            ESP_LOGW(TAG, "REFILL after %s: %ld frames (%ld ms) before a write "
                          "blocked -- phase readings inside this window are not "
                          "DAC-paced",
                     s_refill_why, (long)s_refill_frames,
                     (long)(s_refill_frames * 1000 / (int32_t)stream_rate));
        }
    }
}
#endif

/*
 * The LEDs used to be fed from here, at the DAC, so that they reacted to what
 * this speaker was actually playing rather than to what had merely arrived.
 *
 * They are fed from the receive path now. The objection to that was real -- ~200
 * ms of ring sits between arrival and the speaker, so lights driven from
 * arrival ran that far ahead of the sound -- and it stopped applying when
 * rendering became scheduled: a frame is drawn when the instant it names comes
 * round, not when it was computed, so where it was computed no longer decides
 * when it is seen. What moving it buys is those 200 ms as processing headroom,
 * which is what lets an algorithm cost more than one frame period.
 *
 * Anyone putting this back must put the scheduling back too, or the lights lead
 * the sound by the whole buffer again.
 */
static void write_audio(const uint8_t *pcm, size_t bytes)
{
    dac_write(pcm, bytes);
}

#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
/*
 * When a master-clock instant falls on this board's clock.
 *
 * The same conversion playback uses, against the same slewed offset, so the
 * strip and the speaker are answering to one timeline rather than two. Before
 * the first anchor stream_offset is 0 and this is the identity, which dates the
 * handful of frames produced before a timeline exists into the past -- they are
 * drawn at once, which is the right thing to do with a frame that has no
 * schedule to keep.
 */
static int64_t vis_master_to_local(int64_t master_us)
{
    return sync_to_local(master_us, stream_offset);
}
#endif

/* --------------------------------------------------------------- receiving */

static void probe_task(void *arg)
{
    (void)arg;
    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port = htons(SYNC_PORT),
        .sin_addr.s_addr = inet_addr(MASTER_IP),
    };
    uint32_t seq = 0;

    while (1) {
        time_msg_t msg = { .type = MSG_TIME_REQ, .seq = seq++, .t1 = esp_timer_get_time() };
        sendto(sock, &msg, sizeof(msg), 0, (struct sockaddr *)&dest, sizeof(dest));

        /* Piggyback any track-boundary correction on the same socket. Cleared
         * before sending, so a report landing while this runs is kept rather
         * than overwritten by the clear. */
        if (splice_report_pending) {
            splice_report_pending = false;
            splice_msg_t s = {
                .type = MSG_SPLICE,
                .applied_us = splice_report_us,
                .phase_us = splice_report_phase,
                .applied_med_us = splice_report_med,
            };
            sendto(sock, &s, sizeof(s), 0, (struct sockaddr *)&dest, sizeof(dest));
        }

        vTaskDelay(pdMS_TO_TICKS(PROBE_PERIOD_MS));
    }
}

/*
 * Which clock offset to believe.
 *
 * TSF when it is fresh, the probe estimator otherwise. TSF has no round trip in
 * it, so it carries none of the path asymmetry that is the estimator's error
 * floor -- measured on this hardware the two agree within ~450 us with a stable
 * bias, and TSF's sample-to-sample step is 1-80 us against the estimator's
 * 50-200 us.
 *
 * The estimator stays as the fallback rather than being removed. It works when
 * TSF reads 0 (not associated, no beacon yet, or a hub that does not send
 * MSG_TSF), and losing both at once would leave nothing to anchor on.
 */
static bool clock_offset(int64_t *out, bool *used_tsf)
{
    const int64_t at = tsf_offset_at;
    if (at && esp_timer_get_time() - at < TSF_MAX_AGE_US) {
        *out = tsf_offset_us;
        if (used_tsf) *used_tsf = true;
        return true;
    }
    if (used_tsf) *used_tsf = false;
    return sync_est_offset(&est, out);
}

static void handle_audio(const audio_msg_t *msg)
{
    static uint32_t expect_seq;
    static bool have_seq;
    static int16_t pcm[SBC_MAX_PCM_SAMPLES];

    int64_t offset;
    bool by_tsf = false;
    if (!clock_offset(&offset, &by_tsf)) {
        return;                              /* clock not trusted yet, discard */
    }
    if (msg->format != AUDIO_FMT_SBC) {
        ESP_LOGW(TAG, "unexpected audio format %u -- hub and satellite disagree",
                 msg->format);
        return;
    }

    if (!have_seq || stream_start_local == 0) {
        /*
         * First chunk of a stream: the only moment playback timing is decided,
         * so it must use a clock that can be trusted. Any error here is baked
         * in for the life of the stream.
         *
         * The settling wait applies to the ESTIMATOR only. Three probes produce
         * an offset but not a good one -- minimum-RTT selection needs a full
         * window to find a genuinely uncongested round trip, which costs 2.5 s
         * of silence at every stream start. TSF needs no such wait: it is not
         * built from round trips, so there is no congested sample to average
         * away, and one beacon is as good as ten.
         */
        if (!by_tsf && !sync_est_settled(&est)) {
            static bool told;
            if (!told) {
                told = true;
                ESP_LOGI(TAG, "holding playback until the clock estimate settles");
            }
            return;
        }

        /*
         * Two refusals, both learned from one run where this loop anchored
         * eighteen times in three seconds and every one of them was doomed
         * before playback started.
         *
         * FIRST: a packet whose play_at has already passed cannot be anchored
         * to. The scheduled wait below is what buys the ring its prefill --
         * ~200 ms of audio accumulates while playback holds for its instant --
         * and a negative wait skips it entirely. The run started at "in -90 ms"
         * with `buffer 29 ms`, and every re-anchor after it read worse: -871,
         * -1022, -1234 ms. Anchoring on those produced phase readings of one to
         * 1.25 seconds, which tripped PHASE_INSANE_US, which re-anchored, which
         * reset the ring and threw away the only audio that could have fixed it.
         *
         * The cause was upstream -- the hub's transmit path was refusing sends
         * and what arrived was late and sparse -- and nothing here could have
         * fixed that. But nothing here should have amplified it either. A packet
         * that is already late is evidence about the link, not a timeline.
         *
         * SECOND: one anchor per second. A re-anchor that does not stick is
         * worse than no re-anchor, because xStreamBufferReset() below discards
         * the buffer each time. Refusing for a second parks playback for a
         * second; the alternative measured six seconds of noise.
         *
         * Both are bounded. If every packet is late for ANCHOR_GIVE_UP_US the
         * refusal itself becomes the fault -- a hub whose lead is genuinely
         * shorter than the path, or a clock offset wrong in a way TSF agrees
         * with -- and a satellite that stays silent forever on a stream it could
         * have played badly is not the better failure. Take the packet, say so
         * at ERROR, and let the servo do what it can.
         */
        /*
         * Wait for the playback task to park before touching the ring.
         *
         * A gap-triggered resync sets have_seq false immediately, so the very
         * next packet reaches here -- possibly ~20 ms later, while playback is
         * still draining the ring this is about to reset. xStreamBufferReset()
         * refuses while a task is blocked on the buffer, so the reset would
         * silently not happen while samples_in went to zero underneath it: a
         * stale ring measured against a fresh count, which reads as an insane
         * phase and costs another re-anchor to clear.
         *
         * Bounded, and cannot deadlock: the flag is cleared when the play task
         * parks, which it does within one chunk on seeing the flag, within
         * 500 ms if it is blocked on an empty ring, and unconditionally in the
         * outer loop above whichever route it took.
         */
        if (resync_request) {
            return;
        }

        const int64_t now_local = esp_timer_get_time();
        const int64_t start_local = sync_to_local(msg->play_at, offset);
        static int64_t refuse_since;         /* first refusal of this streak */
        static int64_t last_anchor;

        if (start_local - now_local < ANCHOR_MIN_LEAD_US ||
            (last_anchor && now_local - last_anchor < ANCHOR_MIN_INTERVAL_US)) {
            const bool late = start_local - now_local < ANCHOR_MIN_LEAD_US;
            if (late) n_anchor_late++; else n_anchor_soon++;
            if (refuse_since == 0) {
                refuse_since = now_local;
            }
            if (now_local - refuse_since < ANCHOR_GIVE_UP_US) {
                return;
            }
            ESP_LOGE(TAG, "no anchorable packet for %lld ms (lead %+lld ms) -- "
                          "anchoring on a bad one rather than staying silent",
                     (now_local - refuse_since) / 1000,
                     (start_local - now_local) / 1000);
            anchor_provisional = true;
        } else {
            anchor_provisional = false;   /* this one had the lead it needed */
        }
        refuse_since = 0;
        last_anchor = now_local;
        anchor_at = now_local;
        resync_request = false;   /* may have been set while playback was parked */

        stream_rate = msg->sample_rate ? msg->sample_rate : 44100;
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
        /* Same number the playback task dates its audio with below. The LEDs
         * convert that back to a sample position, so a different rate there
         * would separate the count from the timeline at the difference -- 8.8%
         * for a 48 kHz source against the 44.1 kHz this used to assume. */
        visualiser_set_rate(stream_rate);
        /* A new origin: stream_offset is about to be re-seeded, so any frame
         * already computed and waiting to be drawn is dated against a timeline
         * that stops existing on the next line. */
        visualiser_flush();
#endif
        sbc_decoder_init();
        stream_start_local = start_local;
        stream_offset = offset;
        offset_slew_last = 0;                /* re-seed the slew for this stream */
        /* The visualiser is told nothing here. Anything of its that advances on
         * its own keys off the scheduled instant carried by each chunk, which is
         * already master-clock time and identical on every unit -- so there is
         * no offset for it to be given. */
        samples_in = 0;
        marker_sample = -1;
        phase_head = phase_tail = 0;
        phase_valid = false;
        restart_pos = -1;
        expect_seq = msg->seq;
        have_seq = true;
        xStreamBufferReset(ring);
        n_reanchors++;
        if (by_tsf) n_tsf_used++; else n_tsf_fallback++;
        ESP_LOGI(TAG, "stream start: play_at %lld -> local %lld (in %lld ms) [%s]",
                 msg->play_at, stream_start_local,
                 (stream_start_local - esp_timer_get_time()) / 1000,
                 by_tsf ? "TSF" : "probe estimator");
        /*
         * The first anchor after a rejoin, which is the one that can be built on
         * a clock estimate measured before the outage -- see wifi_down_at. The
         * age is of the NEWEST sample in the estimator's window; if it exceeds
         * the outage then every sample min-RTT selection can choose from
         * predates the drop, and this anchor is as stale as it looks.
         */
        if (rejoined_at) {
            const int64_t age = est_newest_at ? (now_local - est_newest_at) / 1000 : -1;
            ESP_LOGW(TAG, "first anchor after a rejoin: %s, %lld ms since the "
                          "rejoin, newest probe %lld ms old",
                     by_tsf ? "TSF" : "PROBE ESTIMATOR",
                     (now_local - rejoined_at) / 1000, age);
            rejoined_at = 0;
        }
    }

    /*
     * Playback is running on an anchor we already know was bad. Take the first
     * packet that could have been anchored on properly and start again.
     *
     * A give-up anchor is the least-bad answer to "nothing anchorable for five
     * seconds", but it is permanent as it stands: a run anchored at -317 ms
     * lead, read phase +331 ms, and nothing ever re-anchored because that is
     * comfortably inside PHASE_INSANE_US. So the two servos were left to argue
     * about it -- the phase servo pulling the rate up to catch up, the depth net
     * pulling it down because catching up drains the ring -- and the speaker sat
     * a third of a second behind the floor for minutes.
     *
     * None of that is drift, and no rate fixes it. The lateness came from the
     * hub's timeline being displaced, and the moment the hub recovers there is a
     * packet with proper lead in front of it. Re-anchoring on that erases the
     * error in one step instead of asking a 1 ms/s loop to walk it off.
     *
     * The flag is cleared here rather than when the new anchor lands, so one
     * provisional anchor buys exactly one upgrade attempt. If the attempt runs
     * into the give-up path again it sets the flag again, and ANCHOR_MIN_INTERVAL_US
     * bounds how fast that can cycle.
     */
    if (anchor_provisional &&
        sync_to_local(msg->play_at, offset) - esp_timer_get_time() >= ANCHOR_MIN_LEAD_US) {
        anchor_provisional = false;
        n_anchor_upgrades++;
        resync_request = true;
        have_seq = false;
        return;
    }

    /*
     * A lost packet must become silence of exactly the right length. Skipping it
     * would pull every later frame earlier and slide the whole stream against
     * the master -- a permanent error, not a momentary glitch. `frames` tells us
     * how much audio a packet was worth, so a gap can be filled accurately even
     * though SBC packets vary in size.
     */
    if (have_seq && msg->seq > expect_seq) {
        uint32_t missing = msg->seq - expect_seq;
        static const int16_t silence[128 * AUDIO_CHANNELS] = {0};
        uint32_t frames_missing = missing * msg->frames;
        n_gaps++;
        n_gap_frames += frames_missing;

        /*
         * Past a point, filling the gap is the wrong answer.
         *
         * Silence of exactly the right length is right for one or two lost
         * packets: it costs 20 ms of audio and keeps this speaker's position in
         * the timeline, which is the whole reason the fill exists. It stops
         * being right when the gap is an OUTAGE rather than jitter, because the
         * silence has to go somewhere and the ring is not sized for it.
         *
         * Measured, at a stream start where the hub's transmit path dropped ~98
         * datagrams stepping from idle to ~135 a second: the satellite filled
         * 443 ms of silence across twelve gaps, into a ring only 78 ms deep. It
         * came out at `buffer 400 ms` against a 464 ms cap, with ring-full 31 --
         * and a 400 ms ring IS playing late, so phase read +172 ms and the servo
         * spent FIVE MINUTES and nine retunes draining it. The anchor had been
         * perfect: 175 ms of lead, playback started +1 us. All of the damage came
         * from the fill.
         *
         * So beyond GAP_RESYNC_MS this asks for a re-anchor instead. That costs
         * one clean stop and the wait for a packet with proper lead -- a few
         * hundred ms of silence, once -- against minutes of a speaker sitting
         * audibly behind the floor. Both are silence; only one of them ends.
         *
         * have_seq is dropped so the next packet enters the anchor path, where
         * ANCHOR_MIN_LEAD_US decides when playback may start again. The playback
         * task is told separately, because it is mid-stream and will otherwise
         * keep draining a ring that is about to be reset under it.
         */
        if (frames_missing > (uint32_t)((uint64_t)GAP_RESYNC_MS * stream_rate / 1000)) {
            n_gap_resyncs++;
            resync_request = true;
            have_seq = false;
            return;
        }
        /*
         * samples_in must count this silence, and the old loop did not.
         *
         * It is the position every marker and every phase point is recorded
         * against, so audio that goes into the ring uncounted puts all of them
         * one packet (~20 ms) too early -- permanently, and again on the next
         * loss. The playback task then measures its phase against the wrong
         * packet and the servo obediently holds the speaker at that error, so
         * each lost packet moved this unit ~20 ms away from the hub and it never
         * came back.
         *
         * Nothing showed it: the marker pulse fires off the same skewed count,
         * so it lands where the hub's does while the sound and the lights slide.
         *
         * The tail below 128 frames is now inserted too, for the same reason:
         * dropping it left up to 2.9 ms uncounted per loss.
         *
         * Asked of the ring first, rather than discovered 128 frames at a time.
         * A gap is worth `missing` whole packets and a packet is ~20 ms, so a
         * burst loss asks for more than the ring holds: one run's ten-packet
         * gap wanted 8960 frames -- 203 ms, against a 200 ms target depth and
         * the 372 ms ring of the time -- and got 5120 of them in. (RING_BYTES
         * is 80 kB now, so that particular gap would fit; a longer one still
         * would not, and the arithmetic below is what makes the shortfall
         * honest either way.) Pushing until it jams then
         * breaking out reached the same place, but it did so through seventy
         * failing sends, and the shortfall came out as a number nobody could
         * check against the ring's actual free space at the time.
         *
         * What the shortfall MEANS is unchanged and is not fixed here: those
         * frames were owed to the timeline and are not in it, so playback runs
         * that much early until the servo walks it back. Capping only makes the
         * amount honest and the attempt cheap.
         */
        size_t room = xStreamBufferSpacesAvailable(ring);
        uint32_t can_take = (uint32_t)(room / (AUDIO_CHANNELS * sizeof(int16_t)));
        if (can_take < frames_missing) {
            n_gap_short++;
            n_gap_short_frames += frames_missing - can_take;
            frames_missing = can_take;
        }
        while (frames_missing > 0) {
            uint32_t n = frames_missing > 128 ? 128 : frames_missing;
            size_t want = (size_t)n * AUDIO_CHANNELS * sizeof(int16_t);
            size_t sent = xStreamBufferSend(ring, silence, want, 0);
            samples_in += (int32_t)(sent / (AUDIO_CHANNELS * sizeof(int16_t)));
            if (sent < want) {
                break;                       /* raced the playback task; counted above */
            }
            frames_missing -= n;
        }
    } else if (have_seq && msg->seq < expect_seq) {
        return;                              /* duplicate or reorder, drop */
    }
    expect_seq = msg->seq + 1;
    have_seq = true;

    /* Tag before queuing, so the mark lands at the start of this packet's
     * audio and travels through the buffer with it. */
    if (msg->marker && marker_sample < 0) {
        marker_sample = samples_in;
    }

    if (msg->restart && restart_pos < 0) {
        restart_pos = samples_in;
    }

    /* Same idea, for every packet: remember where this audio lands and when it
     * is due, so playback can measure its own phase on arrival. */
    uint32_t next = (phase_head + 1) % PHASE_Q_LEN;
    if (next != phase_tail) {
        phase_q[phase_head].pos = samples_in;
        phase_q[phase_head].play_at = msg->play_at;
        phase_head = next;
    } else {
        n_phase_drop++;      /* see the counter: this used to be silent */
    }

    /* Decode here rather than at the hub: that is the entire point of sending
     * SBC, and it costs a quarter of the airtime. */
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
    uint32_t pkt_frames = 0;      /* frames of this packet already queued */
#endif
    size_t off = 0;
    while (off < msg->payload_len) {
        size_t consumed = 0, samples = 0;
        if (!sbc_decode_frame(msg->payload + off, msg->payload_len - off,
                              &consumed, pcm, &samples)) {
            sbc_decoder_init();              /* resync rather than wedge */
            break;
        }
        if (consumed == 0) {
            break;
        }
        off += consumed;
        size_t want = samples * sizeof(int16_t);
        /* Count what the ring actually took, not what we offered: the same
         * accounting the gap filler above depends on, and a short send here
         * would otherwise bias every later position the other way. */
        size_t sent = xStreamBufferSend(ring, pcm, want, 0);
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
        /*
         * The same audio, dated the same way, fed to the analysis here rather
         * than at the DAC -- see write_audio() for why that moved.
         *
         * play_at is this packet's first sample; interpolate across the frames
         * already taken from it. That is exactly the pairing recorded in the
         * phase queue above, so the LEDs and the phase measurement are reading
         * one timeline rather than two.
         *
         * Fed what the ring TOOK, so the count the block grid rides on stays
         * equal to the audio that will actually be played. Offering what was
         * dropped would separate them at every short send.
         */
        visualiser_feed((const uint8_t *)pcm, (uint32_t)sent,
                        msg->play_at + (int64_t)pkt_frames * 1000000 / stream_rate);
        pkt_frames += (uint32_t)(sent / (AUDIO_CHANNELS * sizeof(int16_t)));
#endif
        if (sent < want) {
            n_ring_full++;
        }
        samples_in += (int32_t)(sent / (AUDIO_CHANNELS * sizeof(int16_t)));
    }
}

static void rx_task(void *arg)
{
    (void)arg;
    /* Sized for the largest message we can receive, which is audio. */
    static uint8_t buf[sizeof(audio_msg_t)];

    while (1) {
        int n = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);
        int64_t t4 = esp_timer_get_time();
        if (n < 1) {
            continue;
        }

        if (buf[0] == MSG_TIME_RSP && n >= (int)sizeof(time_msg_t)) {
            time_msg_t msg;
            memcpy(&msg, buf, sizeof(msg));
            sync_est_add(&est, msg.t1, msg.t2, msg.t3, t4);
            /* The window carries no timestamps of its own, and the whole
             * question about a rejoin is how old the newest sample in it is. */
            est_newest_at = t4;
        } else if (buf[0] == MSG_META && n >= (int)sizeof(meta_msg_t)) {
            const link_meta_t *m = (const link_meta_t *)((const meta_msg_t *)buf)->payload;
            ESP_LOGW(TAG, "TRACK #%" PRIu32 ": \"%s\" - %s [%s]",
                     m->track_id, m->title, m->artist, m->album);
        } else if (buf[0] == MSG_FRAME && n >= (int)FRAME_MSG_BYTES(0)) {
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
            /*
             * An analysis frame the hub computed. Drawn at the instant it
             * names, exactly like one this unit computed itself, so the two
             * sources are interchangeable -- which is what lets the hub run any
             * algorithm at all without it having to be deterministic across
             * units.
             *
             * The length is checked rather than assumed. A hub and a satellite
             * on different builds is the mismatch this protocol is most likely
             * to meet, and reinterpreting a frame of the wrong shape would put
             * garbage on the strip rather than nothing. Said once: it is a
             * property of the pair of builds, so it will not stop happening,
             * and 43 complaints a second would bury everything else.
             */
            const frame_msg_t *fm = (const frame_msg_t *)buf;
            if (fm->len == sizeof(vis_frame_t) &&
                n >= (int)FRAME_MSG_BYTES(fm->len)) {
                vis_frame_t f;
                memcpy(&f, fm->payload, sizeof(f));
                visualiser_submit_frame(&f);
                n_frames_rx++;
            } else {
                static bool told;
                if (!told) {
                    told = true;
                    ESP_LOGE(TAG, "frame of %u bytes, expected %u -- hub and "
                                  "satellite are not the same build",
                             fm->len, (unsigned)sizeof(vis_frame_t));
                }
                n_frames_bad++;
            }
#endif
        } else if (buf[0] == MSG_TSF && n >= (int)sizeof(tsf_msg_t)) {
            /*
             * This is the CLOCK SOURCE, not a measurement. It was one, and this
             * comment still said so long after clock_offset() started
             * preferring it for anchoring and for track_offset() -- the
             * declaration comment on tsf_offset_us is the accurate one. Read
             * everything below as feeding playback directly, because it does.
             *
             * The comparison: both units relate their own TSF to their own
             * esp_timer, and because both TSFs track the same AP counter the
             * difference of those two deltas is the clock offset -- with no
             * round trip in it, so no path asymmetry (see tsf_msg_t).
             */
            tsf_msg_t m;
            memcpy(&m, buf, sizeof(m));

            /*
             * Bracket the esp_timer read between two TSF reads, so the pair
             * carries its own error bar.
             *
             * The two reads are not atomic, and anything that preempts between
             * them lands directly in the delta. The first run showed it: the
             * steps sat at 1 to 80 us and then jumped +333, -296, +360, -355,
             * +521, -515 -- always a jump and an immediate return, which is one
             * bad sample rather than a clock moving. `span` is how long the
             * pair actually took, so a preempted sample can be discarded
             * instead of being read as TSF instability.
             */
            const int64_t tsf_a = esp_wifi_get_tsf_time(WIFI_IF_STA);
            const int64_t my_local = esp_timer_get_time();
            const int64_t tsf_b = esp_wifi_get_tsf_time(WIFI_IF_STA);
            const int64_t span = tsf_b - tsf_a;
            const int64_t my_tsf = (tsf_a + tsf_b) / 2;   /* centred on the timer read */

            /*
             * Say what the first one looked like, whatever it looked like.
             * These used to skip silently on every failure path, so a run with
             * no TSF line could not distinguish "the messages never arrived"
             * from "both counters read zero" from "the estimator was not ready"
             * -- and a measurement that fails invisibly is worse than none.
             */
            static bool announced;
            if (!announced) {
                announced = true;
                ESP_LOGW(TAG, "TSF first message: hub %lld us, ours %lld us%s",
                         m.tsf, my_tsf,
                         (m.tsf && my_tsf) ? "" : "  <-- zero means not available");
            }

            int64_t est_offset;
            if (m.tsf == 0 || my_tsf == 0) {
                static bool told_zero;
                if (!told_zero) {
                    told_zero = true;
                    ESP_LOGW(TAG, "TSF unavailable (hub %lld, ours %lld) -- "
                                  "no comparison possible", m.tsf, my_tsf);
                }
                continue;
            }
            if (!sync_est_offset(&est, &est_offset)) {
                continue;           /* estimator not ready yet; it will be */
            }

            const int64_t tsf_offset = (m.local - m.tsf) - (my_local - my_tsf);

            /* Counted, not enforced -- see n_tsf_wide. This sample is published
             * below exactly as it was before; the counter only says how many
             * would be refused if the span were acted on. */
            if (span > TSF_SPAN_MAX_US) {
                n_tsf_wide++;
            }

            /* Published for anchoring and for the slew. This is the promotion
             * from measurement to source; everything else about the comparison
             * below stays, because it is what would show a regression. */
            tsf_offset_us = tsf_offset;
            tsf_offset_at = my_local;

            /*
             * Rate-limited, and the rate limit comes FIRST so that this line
             * always appears. The previous version discarded wide-span samples
             * before reaching the log, which meant a run where every sample was
             * wide produced total silence -- and the discard counter that would
             * have explained it was only printed on the line that a surviving
             * sample was needed to reach. Twice in this experiment a diagnostic
             * has failed by staying quiet; the fix both times is that the
             * periodic line is unconditional and says what it saw.
             *
             * Nothing is discarded now. `span` is how long the read pair took,
             * so it is the error bar on this sample -- reported beside the
             * value rather than used to hide it, since whether the big steps
             * correlate with a wide span is exactly the question.
             */
            static int64_t last_log_us;
            static int64_t prev_tsf, prev_est;
            static int64_t span_max;
            if (span > span_max) {
                span_max = span;
            }
            if (my_local - last_log_us < (int64_t)CONFIG_DANCEFLOOR_LOG_PERIOD_S * 1000000) {
                continue;
            }
            const int64_t tsf_step = last_log_us ? tsf_offset - prev_tsf : 0;
            const int64_t est_step = last_log_us ? est_offset - prev_est : 0;
            last_log_us = my_local;
            prev_tsf = tsf_offset;
            prev_est = est_offset;

            /* The steps are the point. The estimator swings several ms sample
             * to sample; if TSF has a hardware advantage its step should be
             * microseconds -- beacons are 102.4 ms apart and the local counter
             * free-runs at ~14 ppm between them, so ~1.4 us of drift. */
            ESP_LOGW(TAG, "TSF: est %+lld us | tsf %+lld us | diff %+lld us | "
                          "steps tsf %+lld us, est %+lld us | span %lld us "
                          "(max %lld)",
                     est_offset, tsf_offset, tsf_offset - est_offset,
                     tsf_step, est_step, span, span_max);
            span_max = 0;
        } else if (buf[0] == MSG_AUDIO && n >= (int)AUDIO_MSG_BYTES(0)) {
            handle_audio((const audio_msg_t *)buf);
        }
    }
}

/* ----------------------------------------------------------------- drift */

/*
 * Widest trim the servo may ever ask for. Real drift is ~14 ppm and the buffer
 * safety net asks for 20 Hz, so 100 Hz is already absurd -- anything beyond it
 * is a broken measurement, not a correction.
 */
#define RATE_TRIM_MAX_HZ 100

/*
 * What a retune costs, which nobody has measured.
 *
 * Two effects pull opposite ways and neither is visible to the servo. The
 * channel is DOWN across disable/reconfig/enable, and real time passes with no
 * audio playing, so playback returns that far behind the timeline. Against
 * that, the disable discards the DMA buffer -- audio already counted in
 * samples_played and already fed to the visualiser -- which skips content and
 * pushes the other way.
 *
 * Software can see the first and, by construction, never the second: those
 * frames were counted as played, so every reading derived from samples_played
 * agrees that they were. Only the marker GPIO, which fires when a sample
 * physically reaches the output, can close that gap.
 *
 * These record the first effect directly and the NET at the writer, which is
 * what the servo has to correct. Four retunes scraped from a session put the
 * net somewhere between +5 and +22 ms; that is a range, not a number, because
 * phase wanders by several ms on its own between the 5 s log ticks.
 */
static volatile int32_t retune_phase_before;
static volatile bool    retune_watch;      /* playback reports the next reading */
static volatile int64_t retune_outage_us;

/*
 * When the retune finished, and how many crossings have been narrated since.
 *
 * MEASUREMENT ONLY -- the servo still withholds exactly one reading, so this
 * build behaves identically to the last and merely says more.
 *
 * The bench numbers behind the one-shot withholding were taken here: 19
 * same-rate retunes, net +4.4 ms against a 3.6 ms outage, every one positive,
 * the crossing landing 1-22 ms after the retune and inside the refill every
 * time. Crossings arrive one per packet, ~20 ms apart, so one withheld reading
 * covers perhaps the first of a disturbance that reaches 22 ms -- and whatever
 * is left goes to the servo as position error, so each retune injects what the
 * next one corrects. These lines say how far the tail actually reaches, which
 * is what sizes a settle window instead of guessing one.
 */
static volatile int64_t retune_done_at;
static volatile uint8_t retune_tail_left;

/*
 * Held across a retune, and the playback task parks on it.
 *
 * i2s_channel_write() returns IMMEDIATELY once the channel is disabled -- it
 * does not block, and dac_write() did not look at the return value -- so for
 * the whole outage the play task ran flat out: pulling chunks from the ring,
 * counting them in samples_played, feeding them to the visualiser, and throwing
 * them at a channel that was not running. Milliseconds of outage cost tens of
 * milliseconds of buffer.
 *
 * Measured on hardware: a 7.7 ms outage produced a +42 ms phase step, 5432
 * bytes overflowed the visualiser's buffer, and it re-aligned nine times in one
 * window. The short outages, where the task had less time to spin, cost +4 ms.
 *
 * The hub has had this since "a measured 54 ms correction cost 177 ms of
 * buffer". It was never ported here, and every satellite retune has been paying
 * for it since.
 */
static volatile bool retuning;

#if !CONFIG_DANCEFLOOR_USE_INTERNAL_DAC
static void retune_output(uint32_t hz)
{
    /*
     * Nothing the servo computes may panic the speaker.
     *
     * This aborted the board with ESP_ERROR_CHECK when a wrapped phase error
     * produced a rate of ~4.29e9 Hz: the servo arithmetic went wrong, and what
     * the room heard was a satellite rebooting. A refused retune costs sync; an
     * abort costs the unit. Clamp, log, carry on playing at the current rate.
     */
    const int64_t low  = (int64_t)stream_rate - RATE_TRIM_MAX_HZ;
    const int64_t high = (int64_t)stream_rate + RATE_TRIM_MAX_HZ;
    if ((int64_t)hz < low || (int64_t)hz > high) {
        n_retunes_bad++;
        ESP_LOGE(TAG, "refusing a %" PRIu32 " Hz retune (nominal %" PRIu32
                      ") -- the servo input is not trustworthy", hz, stream_rate);
        return;
    }

    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(hz);
    const int64_t down_at = esp_timer_get_time();

    /* Park playback before taking the channel down, and count the park as part
     * of the outage -- audio is stopped for it just as surely. */
    retuning = true;
    vTaskDelay(pdMS_TO_TICKS(2));

    esp_err_t err = i2s_channel_disable(i2s_tx);
    if (err == ESP_OK) {
        err = i2s_channel_reconfig_std_clock(i2s_tx, &clk);
        /* Re-enable whatever happened: leaving the channel down stalls playback
         * silently, which reads as a dead board rather than a failed trim. */
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
    retune_outage_us = esp_timer_get_time() - down_at;
    n_retunes++;
    ESP_LOGW(TAG, "output clock retuned %" PRIu32 " -> %" PRIu32 " Hz, channel down %lld us",
             tx_rate, hz, retune_outage_us);
    tx_rate = hz;

    /* Ask playback to report the first phase it measures after this, so the net
     * cost is one printed number rather than a difference between two 5 s log
     * ticks with several ms of wander in each. */
    retune_phase_before = phase_err_us;
    retune_watch = true;
    /* Ordered after the two above: the play task reads them together and this
     * is what arms the narration. See retune_done_at. */
    retune_tail_left = 3;
    retune_done_at = esp_timer_get_time();

    /*
     * Nothing to tell the visualiser. It counts what ARRIVES now, and a retune
     * disturbs playback rather than arrival -- so the block grid it rides on is
     * untouched by anything that happens here.
     *
     * It used to need telling, on the reasoning that disabling the channel
     * discarded the DMA buffer, which the playback task had already counted as
     * played and already fed onward. Two things retired that: the analysis is no
     * longer on the playback path at all, and the REFILL instrument showed the
     * descriptors are not discarded in the first place.
     */
}
#endif

/*
 * Hold the buffer level steady, which is what keeps this speaker in step with
 * the hub.
 *
 * The start instant is set once from play_at. After that this board's DAC runs
 * on its own crystal -- measured ~14 ppm from the hub's, which is 0.8 ms of
 * drift per minute and 50 ms per hour. Inaudible at first, then unmistakable
 * slapback. Aligning the start is not enough on its own.
 *
 * The buffer level is the integral of the rate error, so nulling it matches the
 * rates and preserves the phase established at start. Correcting over ~40 s
 * keeps every adjustment far below the ~1% shift a listener could notice.
 *
 * Same approach as the hub's ring servo; the difference is only which clock is
 * being trimmed.
 */
static void drift_task(void *arg)
{
    (void)arg;
    int32_t err_ema = 0;
    bool err_ema_valid = false;
    /*
     * Windows to wait after a retune before considering another. The buffer
     * takes tens of seconds to respond, and acting again before it has is how
     * a servo ends up chasing its own corrections.
     */
    int cooldown = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        const uint32_t heap_now = esp_get_free_heap_size();
        if (heap_now < heap_min_window) {
            heap_min_window = heap_now;
        }

        /* Said once, and within 5 s of the fact rather than at the next soak
         * line -- see the hub's copy. */
        static uint32_t alloc_fail_told;
        if (n_alloc_fail != alloc_fail_told) {
            alloc_fail_told = n_alloc_fail;
            ESP_LOGE(TAG, "ALLOCATION FAILED %" PRIu32 " time(s): largest request %"
                          PRIu32 " B (caps 0x%" PRIx32 "), heap %" PRIu32
                          " free, largest block %u",
                     n_alloc_fail, alloc_fail_size, alloc_fail_caps, heap_now,
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
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
        static uint32_t gaps_told, gap_frames_told, gap_short_told,
                        gap_short_frames_told, ring_full_told,
                        anchor_late_told, anchor_soon_told, gap_resyncs_told,
                        upgrades_told;
        const uint32_t gaps_now = n_gaps, gap_frames_now = n_gap_frames,
                       gap_short_now = n_gap_short,
                       gap_short_frames_now = n_gap_short_frames,
                       ring_full_now = n_ring_full,
                       anchor_late_now = n_anchor_late,
                       anchor_soon_now = n_anchor_soon,
                       gap_resyncs_now = n_gap_resyncs,
                       upgrades_now = n_anchor_upgrades;
        if (gaps_now != gaps_told || ring_full_now != ring_full_told ||
            anchor_late_now != anchor_late_told || anchor_soon_now != anchor_soon_told ||
            gap_resyncs_now != gap_resyncs_told || upgrades_now != upgrades_told) {
            ESP_LOGW(TAG, "RX 5s: gaps %" PRIu32 " (%" PRIu32 " ms silence, %"
                          PRIu32 " short by %" PRIu32 " ms) | ring-full %" PRIu32
                          " | too big to fill %" PRIu32 " | upgrades %" PRIu32
                          " | anchors refused %" PRIu32 " late, %" PRIu32 " too soon",
                     gaps_now - gaps_told,
                     (gap_frames_now - gap_frames_told) * 1000 / stream_rate,
                     gap_short_now - gap_short_told,
                     (gap_short_frames_now - gap_short_frames_told) * 1000 / stream_rate,
                     ring_full_now - ring_full_told,
                     gap_resyncs_now - gap_resyncs_told,
                     upgrades_now - upgrades_told,
                     anchor_late_now - anchor_late_told,
                     anchor_soon_now - anchor_soon_told);
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
                     " | leds %s hop %d (rx %" PRIu32 ", bad %" PRIu32 ")",
                     (unsigned long long)(esp_timer_get_time() / 1000000),
                     esp_get_free_heap_size(), esp_get_minimum_free_heap_size(),
                     heap_win == UINT32_MAX ? 0 : heap_win,
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT),
                     hw_play, hw_drift, n_underruns, n_reanchors, n_splices,
                     n_retunes, n_retunes_bad, n_gaps, n_gap_short, n_gap_resyncs, n_ring_full,
                     n_anchor_upgrades,
                     n_anchor_late + n_anchor_soon, n_wifi_drops, n_alloc_fail,
                     (tsf_offset_at && esp_timer_get_time() - tsf_offset_at < TSF_MAX_AGE_US)
                         ? "TSF" : "probe",
                     n_tsf_used, n_tsf_fallback, n_tsf_wide,
                     n_phase_drop, n_short_reads, n_short_frames,
                     visualiser_source_name(), visualiser_hop(),
                     n_frames_rx, n_frames_bad);

#if CONFIG_DANCEFLOOR_WIFI_LOGS
            /* The structured twin of the line above, for the collector's CSV.
             * Every field is already in scope here; the role aliases are
             * documented on health_msg_t in sync_proto.h. */
            static uint32_t health_seq;
            health_msg_t h;
            memset(&h, 0, sizeof h);
            h.type = MSG_HEALTH;
            h.role = LOG_ROLE_SAT;
            const bool tsf_now = tsf_offset_at &&
                esp_timer_get_time() - tsf_offset_at < TSF_MAX_AGE_US;
            h.clock_src = tsf_now ? 1 : 0;
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

        if (stream_start_local == 0) {
            continue;
        }

#if CONFIG_DANCEFLOOR_RETUNE_BENCH_S > 0 && !CONFIG_DANCEFLOOR_USE_INTERNAL_DAC
        /*
         * Bench: retune to the rate we are already at. Nothing about the audio
         * changes, so whatever the RETUNE COST line then reports is the cost of
         * retuning alone -- no rate change, no drift, no track boundary in the
         * way. Run it on one unit and leave the other as the reference.
         */
        static int bench_left;
        if (--bench_left <= 0) {
            bench_left = (CONFIG_DANCEFLOOR_RETUNE_BENCH_S + 4) / 5;
            ESP_LOGW(TAG, "BENCH: forcing a same-rate retune at %" PRIu32 " Hz", tx_rate);
            retune_output(tx_rate);
            continue;                        /* do not also servo this window */
        }
#endif

        size_t filled = RING_BYTES - xStreamBufferSpacesAvailable(ring);
        int32_t target = (int32_t)(RING_TARGET_MS *
                                   (stream_rate * AUDIO_CHANNELS * 2 / 1000));
        int32_t err_frames = ((int32_t)filled - target) / (AUDIO_CHANNELS * 2);

        /*
         * Smooth before acting, and separate two things that look identical in a
         * single reading:
         *
         *   jitter -- delivery is bursty, so the level swings +-40 ms with no
         *             trend. Correcting for it would retune constantly, and each
         *             retune is an audible click.
         *   drift  -- the crystals genuinely differ (~14 ppm measured), so the
         *             level walks steadily in one direction.
         *
         * Averaging kills the first and leaves the second. The old code coped by
         * using a wide deadband instead, which meant real drift could reach
         * ~60 ms before anything happened -- clearly audible echo, and about 75
         * minutes away at 14 ppm. Fine in a short test, wrong over an evening.
         */
        /* Smoothed phase error is what the servo acts on now. Buffer depth is
         * kept only as a safety net against underrun or overflow, which phase
         * control alone would not notice until it was too late. */
        int32_t ph = phase_valid ? phase_err_us : 0;
        if (phase_stepped) {
            phase_stepped = false;
            err_ema_valid = false;       /* history describes a different world */
        }
        err_ema = err_ema_valid ? (err_ema * 3 + ph) / 4 : ph;
        err_ema_valid = true;

        /* Once per log period, not every window. The servo above still runs at
         * 5 s and still sees every sample; it just stops narrating. */
        static int status_left;
        if (--status_left <= 0) {
            status_left = CONFIG_DANCEFLOOR_LOG_PERIOD_S / 5;
            ESP_LOGI(TAG, "buffer %lu ms | phase %+ld us (smoothed %+ld us)",
                     (unsigned long)(filled * 1000 / (stream_rate * AUDIO_CHANNELS * 2)),
                     (long)ph, (long)err_ema);
        }

#if CONFIG_DANCEFLOOR_USE_INTERNAL_DAC
        /* The internal DAC's rate is fixed at creation, so there is nothing to
         * trim. Fine for bench listening, not for a unit that must stay in sync
         * for hours -- use an I2S DAC for that. */
        (void)err_frames;
#else
        /*
         * Late (positive error) means we are behind the timeline, so play
         * faster.
         *
         * Spread over ~100 s, not 40. The buffer takes tens of seconds to
         * respond, so a 40 s loop was still correcting after the error had gone
         * and sailed past it -- both units converged to near zero then
         * overshot to +10 ms and oscillated. Real drift is only ~0.8 ms per
         * minute, so the loop can afford to be much gentler than the
         * disturbance it corrects.
         */
        int32_t adj = (int32_t)((int64_t)err_ema * stream_rate / 100000000LL);
        /* Belt and braces against the arithmetic above, which has produced
         * -138000 once already. retune_output() refuses anything wilder, but
         * the number should not get that far in the first place. */
        if (adj >  RATE_TRIM_MAX_HZ) adj =  RATE_TRIM_MAX_HZ;
        if (adj < -RATE_TRIM_MAX_HZ) adj = -RATE_TRIM_MAX_HZ;

        /*
         * Safety net: if the buffer is heading for empty or full, that matters
         * more than phase.
         *
         * Held off for DEPTH_NET_HOLD_US after an anchor, because for that
         * stretch the depth is not evidence of anything the net exists to
         * catch.
         *
         * A fresh stream starts below target by construction. RING_TARGET_MS is
         * 200, matching the hub's LEAD_US, but the deepest prefill an anchor can
         * ever buy is that lead MINUS transit -- the scheduled wait is the only
         * thing that fills the ring before playback begins, so the best measured
         * start was 154 ms and a 107 ms one is unremarkable. The reading is
         * lower still in the first moments, because playback has begun consuming
         * while the rest of the stream is in flight: a run read `buffer 40 ms`
         * 100 ms after playback started, from a 107 ms prefill.
         *
         * The net fired on exactly that, dropping the clock 44100 -> 44080 to
         * rescue a ring that was not in trouble. The stream then spent 110 s and
         * six retunes walking off the phase excursion it caused -- peak +48 ms,
         * with the visualiser rendering 7% of its frames late while the sound
         * ran behind the timeline the lights were drawn on.
         *
         * 20 s is four servo windows and matches the retune cooldown, by which
         * point the phase measurement is trustworthy and is the better input
         * anyway. Nothing about underrun protection is given up here: an
         * actually empty ring is caught by the playback task's 500 ms receive
         * timeout, which is a different mechanism and still armed.
         */
        const int64_t since_anchor = anchor_at ? esp_timer_get_time() - anchor_at
                                               : INT64_MAX;
        if (since_anchor < DEPTH_NET_HOLD_US) {
            /* say nothing; the buffer line above already prints the depth */
        } else if (err_frames * 1000 / (int32_t)stream_rate < -120) {
            adj = -20;                       /* nearly empty: slow down */
        } else if (err_frames * 1000 / (int32_t)stream_rate > 120) {
            adj = 20;                        /* nearly full: speed up */
        }
        uint32_t desired = (uint32_t)((int32_t)stream_rate + adj);
        /*
         * Deadband, stated in phase error rather than in rate.
         *
         * It used to be tx_rate/5000, described in a comment as "~8 ms of
         * accumulated drift before a correction". That read 0.02% of the sample
         * rate as if it were milliseconds: 8 Hz of threshold is really 8e8/44100
         * = ~20 ms of phase, per unit and in either direction.
         *
         * Affordable at 7 ms because a retune is cheap now -- measured at 1 to 7
         * ms, essentially just the channel outage. It was not while this task
         * spun through the ring during that outage; see PHASE_DEADBAND_US.
         */
        if (!phase_valid) {
            continue;                        /* nothing measured yet */
        }
        int32_t deadband = (int32_t)((int64_t)PHASE_DEADBAND_US * stream_rate / 100000000LL);
        if (deadband < 1) {
            deadband = 1;
        }
        if (cooldown > 0) {
            cooldown--;
        } else if (desired > tx_rate + (uint32_t)deadband ||
                   desired < tx_rate - (uint32_t)deadband) {
            retune_output(desired);
            cooldown = 4;        /* ~20 s, against a 40 s correction time */
        }
#endif
    }
}

/* --------------------------------------------------------------- playback */

/*
 * Keep the local -> master conversion current.
 *
 * The offset measured at anchoring goes stale at whatever the two crystals
 * differ by -- 10.6 ppm on our boards, so 38 ms in an hour (docs/clock-sync.md
 * §9). The phase measurement below converts local time to master time with it,
 * so holding it fixed biases that measurement by exactly the drift, and the
 * servo then faithfully parks the speaker at the growing error instead of
 * removing it. The drift the servo exists to correct was being fed back in as
 * its own reference.
 *
 * Invisible in the logs, which is why it survived: the marker pulse is derived
 * from the same conversion, so the cross-unit measurement reads correct while
 * the sound and the lights slide apart.
 *
 * Slewed, never stepped. Minimum-RTT selection moves the raw estimate by a few
 * ms as one probe replaces another in the window, and handing that straight to
 * the servo looks exactly like a real position error. The limit below is ~15x
 * the drift it has to follow, so drift is tracked with room to spare while
 * estimator noise averages out over tens of seconds.
 *
 * Nothing jumps as a result. This moves only where the servo believes it is;
 * the servo answers in sample rate, over ~100 s, as it always did.
 */
#define OFFSET_SLEW_PPM 200

static void track_offset(void)
{
    int64_t measured;
    if (!clock_offset(&measured, NULL)) {
        return;
    }

    const int64_t now = esp_timer_get_time();
    if (offset_slew_last == 0) {
        offset_slew_last = now;              /* first look at this stream */
        return;
    }
    const int64_t limit = (now - offset_slew_last) * OFFSET_SLEW_PPM / 1000000;
    if (limit == 0) {
        return;                              /* too soon to move; let dt build */
    }
    offset_slew_last = now;

    int64_t diff = measured - stream_offset;
    if (diff >  limit) diff =  limit;
    if (diff < -limit) diff = -limit;
    stream_offset += diff;
}

static void play_task(void *arg)
{
    (void)arg;
    static uint8_t chunk[AUDIO_CHUNK_BYTES];

    while (1) {
        if (stream_start_local == 0) {
            /* Parked, by whichever route -- resync, underrun or a changed
             * timeline. The flag has been served either way, and clearing it
             * here rather than only where it is consumed is what lets the
             * anchor path below wait on it without being able to deadlock. */
            resync_request = false;
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        /* Hold the first sample until its moment arrives. After this, I2S paces
         * everything: i2s_channel_write blocks once the DMA buffers are full. */
        int64_t wait = stream_start_local - esp_timer_get_time();
        if (wait > 2000) {
            vTaskDelay(pdMS_TO_TICKS((wait - 2000) / 1000));
        }
        while (esp_timer_get_time() < stream_start_local) {
            /* spin the last stretch, as in the M4 blink task */
        }
        int64_t actual_master = esp_timer_get_time() + stream_offset;
        int64_t sched_master = stream_start_local + stream_offset;
        ESP_LOGI(TAG, "playback started: scheduled %lld, actual %lld (%+lld us) [master]",
                 sched_master, actual_master, actual_master - sched_master);
        /*
         * samples_played counts from the first sample played, which is the
         * first sample queued after the ring was reset on anchoring. The two
         * counters therefore share an origin -- do NOT reset samples_in here.
         * It has legitimately been counting the audio buffered during the wait,
         * and zeroing it shifts every marker by the buffer depth.
         */
        int32_t samples_played = 0;
        /* Every reading in it was measured against the stream this anchor
         * replaces, so none of them describes where this unit now is. */
        sync_phase_reset(&phase_hist);
        /*
         * Measure the DMA prefill at every playback START, not only after a
         * retune. The channel has been draining while this task was parked, so
         * it is empty here for exactly the same reason it is empty after a
         * disable -- and i2s_channel_write() does not block until the
         * descriptors are full, so the first writes return at memory speed and
         * samples_played advances by the whole DMA depth against a wrote_at
         * that has barely moved. Every phase reading dated from that window is
         * measured against a reference the DAC is not pacing.
         *
         * This is the same mechanism the `retuning` park exists to prevent, and
         * clock-sync.md records it costing +42, +43 and +50 ms there before the
         * park existed. Nothing guards it here.
         *
         * It is very likely the "-42 ms (hub), -26 ms (satellite)" startup phase
         * in clock-sync.md §8 -- a 16 ms difference between two units that then
         * takes ~45 s to walk off, on a cold start, on both units at once. On a
         * reconnect only this unit restarts, against a hub already servoed to
         * zero, which is a different and much easier situation.
         *
         * MEASUREMENT ONLY: the REFILL line reports, nothing withholds. Whether
         * to withhold is the next question and this is what sizes it.
         */
#if !CONFIG_DANCEFLOOR_USE_INTERNAL_DAC
        s_refill_active = true;
        s_refill_frames = 0;
        s_refill_why = "start";
#endif
        /* When the DAC last accepted a chunk -- what the phase reading is dated
         * against. See the hub's copy for why it is not a clock read taken in
         * the phase loop. Seeded so the first pass has a sane value. */
        int64_t wrote_at = esp_timer_get_time();

        bool was_retuning = false;

        while (1) {
            if (retuning) {
                /* Do not pull from the ring while the channel is down -- the
                 * writes would return instantly and drain it. */
                was_retuning = true;
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
            /*
             * The receive task found a gap too large to fill and wants a clean
             * restart -- see GAP_RESYNC_MS. Leave by the same door as an
             * underrun, so the anchor path owns the ring reset rather than
             * racing this loop for it. Noticed within one chunk (~5.8 ms),
             * comfortably inside the ~20 ms until the next packet arrives.
             *
             * Not logged here. This task is the audio path, and putting an
             * ESP_LOGW on it is the mistake the RX counters exist to undo;
             * drift_task narrates n_gap_resyncs within 5 s instead.
             */
            if (resync_request) {
                resync_request = false;
                stream_start_local = 0;
                phase_valid = false;
                phase_stepped = true;
                break;
            }
            if (was_retuning) {         /* see REFILL_FAST_US */
                was_retuning = false;
                s_refill_active = true;
                s_refill_frames = 0;
                s_refill_why = "retune";
            }
            hw_play = uxTaskGetStackHighWaterMark(NULL);   /* only valid in-task */
            size_t got = xStreamBufferReceive(ring, chunk, sizeof(chunk), pdMS_TO_TICKS(500));
            if (got == 0) {
                n_underruns++;
                ESP_LOGW(TAG, "underrun, waiting for a new stream");
                stream_start_local = 0;
                break;
            }
            if (got < sizeof(chunk)) {
                memset(chunk + got, 0, sizeof(chunk) - got);
                n_short_reads++;
                n_short_frames += (uint32_t)((sizeof(chunk) - got)
                                             / (AUDIO_CHANNELS * sizeof(int16_t)));
            }
            /* Before measuring anything against the master clock, make sure the
             * conversion still describes it. */
            track_offset();

            /* Has playback reached a recorded packet boundary? If so, compare
             * now against when that sample was due. */
            bool timeline_changed = false;
            while (phase_tail != phase_head && samples_played >= phase_q[phase_tail].pos) {
                int64_t due = phase_q[phase_tail].play_at;
                /*
                 * Dated by where the crossing HAPPENED, not where the loop
                 * noticed it. samples_played moves AUDIO_FRAMES at a time --
                 * 5.8 ms at 44.1 kHz -- so the crossing is up to a chunk in the
                 * past by an amount that depends on where pos falls on the
                 * chunk grid, which is uncorrelated noise straight into the
                 * servo's only input. The overshoot is known, and writes are
                 * paced by the DAC, so the correction is exact rather than a
                 * filter. See the hub's copy for what this cost there.
                 */
                int32_t overshoot = samples_played - phase_q[phase_tail].pos;
                /* Capped at one chunk: a splice steps samples_played by up to
                 * MAX_SPLICE_MS at once and those frames were skipped rather
                 * than played, so anything the jump carried us past cannot be
                 * dated this way. Same guard as the hub. */
                if (overshoot > (int32_t)AUDIO_FRAMES) {
                    overshoot = (int32_t)AUDIO_FRAMES;
                }
                /* Dated from when the DAC last took a chunk, not from a clock
                 * read here -- the write is the only DAC-paced event in this
                 * loop, and everything between it and this line is unpaced.
                 * See the hub's copy, which is where that mattered. */
                /* The crossing instant on THIS unit's clock. now_master is the
                 * same instant converted; the local form is what dates the
                 * crossing against a retune, which is a local event. */
                int64_t crossed_at = wrote_at
                                   - (int64_t)overshoot * 1000000 / stream_rate;
                int64_t now_master = crossed_at + stream_offset;
                int64_t err = now_master - due;
                /*
                 * Seconds of error is not drift and not jitter. It means the
                 * stamps are being issued against a different clock origin than
                 * the one playback anchored to -- the hub rebooted or was
                 * reflashed while this unit kept playing -- and no servo can
                 * correct that, because there is nothing wrong with the rate.
                 *
                 * It used to be cast straight into an int32: an hour of error
                 * wrapped to -699 seconds, the smoothing overflowed on top of
                 * it, and the servo asked for a 4.29 GHz sample rate, which
                 * aborted the board. Re-anchor instead, which is the one action
                 * that actually fixes it -- it re-reads the offset against the
                 * clock the hub is really using now.
                 */
                if (err > PHASE_INSANE_US || err < -PHASE_INSANE_US) {
                    ESP_LOGE(TAG, "phase %lld us -- not the timeline we anchored to, "
                                  "re-anchoring", err);
                    timeline_changed = true;
                    break;
                }
                /* The first reading after a retune is a transient -- logged,
                 * then thrown away rather than handed to the servo. See the
                 * hub's copy for what the outage figure actually covers and
                 * for the bench numbers that came off this unit. */
                const int64_t since_retune = retune_done_at
                                           ? crossed_at - retune_done_at : -1;
                if (retune_watch) {
                    retune_watch = false;
                    ESP_LOGW(TAG, "RETUNE COST: phase %+ld -> %+lld us (net %+lld), "
                                  "channel was down %lld us -- withheld from the "
                                  "servo, crossed %lld us after the retune",
                             (long)retune_phase_before, err, err - retune_phase_before,
                             retune_outage_us, since_retune);
                } else {
                    /*
                     * Narrated but NOT withheld -- these reach the servo exactly
                     * as they did before, so behaviour is unchanged and only the
                     * log says more. Whether they SHOULD be withheld is the
                     * question these lines exist to answer.
                     */
                    if (retune_tail_left) {
                        retune_tail_left--;
                        ESP_LOGW(TAG, "RETUNE TAIL: phase %+lld us at %lld us after "
                                      "the retune (net %+lld from before it)",
                                 err, since_retune, err - retune_phase_before);
                    }
                    phase_err_us = (int32_t)err;
                    phase_valid = true;
                    sync_phase_push(&phase_hist, (int32_t)err);
                }
                phase_tail = (phase_tail + 1) % PHASE_Q_LEN;
            }
            if (timeline_changed) {
                /* Same exit as an underrun: handle_audio() re-anchors on the
                 * next packet, which re-seeds stream_offset from a current
                 * estimate rather than one describing a hub that no longer
                 * exists. Drop the smoothing with it -- every sample in it was
                 * measured against the old origin. */
                stream_start_local = 0;
                phase_valid = false;
                phase_stepped = true;
                break;
            }

            /*
             * Track boundary reached: snap phase to zero rather than letting the
             * servo walk it off over ~45 s. Skipping or inserting audio is
             * inaudible here and nowhere else.
             */
            int32_t rp = restart_pos;
            if (rp >= 0 && samples_played >= rp) {
                restart_pos = -1;
                int32_t max_frames = (int32_t)stream_rate * MAX_SPLICE_MS / 1000;
                /* Same guard as the hub: phase_err_us survives a re-anchor, so
                 * a boundary reached before the first measurement of the new
                 * stream would splice on a number describing the old one. */
                int32_t adj = phase_valid
                    ? (int32_t)((int64_t)phase_err_us * stream_rate / 1000000) : 0;
                if (adj > max_frames)  adj = max_frames;
                if (adj < -max_frames) adj = -max_frames;
                int32_t applied = 0;      /* what the splice actually moved */

                /*
                 * SHADOW: the correction the median of the last few readings
                 * would have asked for, clamped identically so it subtracts
                 * meaningfully against the hub's. Reported to the hub, acted on
                 * by nothing -- the splice above still runs on phase_err_us.
                 * Same computation as the hub's copy, deliberately.
                 */
                int32_t med_us = 0;
                if (phase_valid && sync_phase_median(&phase_hist, &med_us)) {
                    int32_t med_adj = (int32_t)((int64_t)med_us * stream_rate / 1000000);
                    if (med_adj > max_frames)  med_adj = max_frames;
                    if (med_adj < -max_frames) med_adj = -max_frames;
                    splice_report_med = (int32_t)((int64_t)med_adj * 1000000 / stream_rate);
                } else {
                    splice_report_med = 0;
                }

                if (adj > 0) {
                    /* Late: discard input so playback jumps forward in content.
                     * samples_played tracks input consumed, so it advances too.
                     *
                     * Into its own buffer, not into chunk: chunk is holding the
                     * audio read at the top of this pass, which has not been
                     * played yet. Discarding into it threw that away and played
                     * the tail of the skipped region in its place -- 5.8 ms of
                     * the wrong audio at every boundary. Nothing drifted, since
                     * both reads are counted, but it is not what the splice is
                     * supposed to do. */
                    static uint8_t discard[AUDIO_CHUNK_BYTES];
                    int32_t left = adj;
                    while (left > 0) {
                        size_t want = (size_t)(left > (int32_t)AUDIO_FRAMES ? AUDIO_FRAMES : left)
                                      * AUDIO_CHANNELS * sizeof(int16_t);
                        size_t got2 = xStreamBufferReceive(ring, discard, want, 0);
                        if (got2 == 0) break;
                        left -= got2 / (AUDIO_CHANNELS * sizeof(int16_t));
                    }
                    samples_played += (adj - left);
                    applied = adj - left;
                    ESP_LOGW(TAG, "track boundary: skipped %ld ms to null phase",
                             (long)(applied * 1000 / (int32_t)stream_rate));
                    phase_stepped = true;
                } else if (adj < 0) {
                    /* Early: emit silence so the timeline catches up with us. */
                    static const uint8_t quiet[AUDIO_CHUNK_BYTES] = {0};
                    int32_t left = -adj;
                    while (left > 0) {
                        int32_t n = left > (int32_t)AUDIO_FRAMES ? (int32_t)AUDIO_FRAMES : left;
                        write_audio(quiet, (size_t)n * AUDIO_CHANNELS * sizeof(int16_t));
                        left -= n;
                    }
                    applied = adj;
                    ESP_LOGW(TAG, "track boundary: inserted %ld ms to null phase",
                             (long)(-applied * 1000 / (int32_t)stream_rate));
                    phase_stepped = true;
                }

                /*
                 * Hand the correction to the probe task to report. Not sent
                 * from here: a sendto() in the playback path is exactly the
                 * kind of thing that costs a buffer, and this is not urgent --
                 * the hub only wants it to print one line per track.
                 */
                if (applied != 0) {
                    n_splices++;
                    /* Same reason phase_stepped is set above: every reading in
                     * the history was taken before this unit moved. */
                    sync_phase_reset(&phase_hist);
                }
                splice_report_us = (int32_t)((int64_t)applied * 1000000 / stream_rate);
                splice_report_phase = phase_valid ? phase_err_us : 0;
                splice_report_pending = true;
                /*
                 * The visualiser is not told either. A splice moves audio around
                 * WITHIN the timeline to correct this unit's position in it; the
                 * timeline itself, which is what every frame is dated against and
                 * drawn on, does not move. Arrival is untouched.
                 */
            }

            int32_t mark = marker_sample;
            if (mark >= 0 && samples_played >= mark) {
#if CONFIG_DANCEFLOOR_ENABLE_MARKER
                /* 200 us of busy-wait in the playback path. Worth it while a
                 * hub is wired to the other end and nothing else can measure
                 * what reaches the speaker; pure cost once the wire is off. */
                gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 1);
                esp_rom_delay_us(MARKER_PULSE_US);
                gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 0);
#endif
                marker_sample = -1;
            }
            samples_played += AUDIO_FRAMES;

            write_audio(chunk, sizeof(chunk));
            /* Immediately: it is the instant the next pass dates its phase
             * reading from. */
            wrote_at = esp_timer_get_time();
        }
    }
}

/* Printed at boot so a log immediately identifies which build produced it --
 * compile time and ELF hash both change on every rebuild. Saves guessing
 * whether a reflash actually landed. */
static void log_build_stamp(const char *tag)
{
    const esp_app_desc_t *d = esp_app_get_description();
    char sha[17] = {0};
    for (int i = 0; i < 8; i++) {
        sprintf(sha + i * 2, "%02x", d->app_elf_sha256[i]);
    }
    ESP_LOGW(tag, "BUILD %s %s  elf:%s", d->date, d->time, sha);
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Before anything else allocates, so a failure during WiFi or socket setup
     * is caught too -- that is the phase with the largest single requests. */
    ESP_ERROR_CHECK(heap_caps_register_failed_alloc_callback(on_alloc_failed));

    sync_est_init(&est);
    /* Link check only for now -- the SBC receive path lands in the next stage. */
    ESP_LOGI(TAG, "SBC decoder init: %s", sbc_decoder_init() ? "ok" : "FAILED");
    ring = xStreamBufferCreate(RING_BYTES, AUDIO_CHUNK_BYTES);
    assert(ring);

    wifi_start_sta();
    socket_start();
    /* Mirror ESP_LOG lines to the hub (and a structured HEALTH, below). No-op
     * and compiles to nothing unless CONFIG_DANCEFLOOR_WIFI_LOGS is set. */
    wifi_log_init(LOG_ROLE_SAT, MASTER_IP);
    i2s_start(44100);
    tx_rate = 44100;

#if CONFIG_DANCEFLOOR_ENABLE_MARKER
    gpio_config_t marker = {
        .pin_bit_mask = 1ULL << CONFIG_DANCEFLOOR_MARKER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&marker));
    ESP_LOGI(TAG, "sync marker on GPIO %d -- bench instrument, nothing corrects on it",
             CONFIG_DANCEFLOOR_MARKER_GPIO);
#endif

#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
    visualiser_start();
    /*
     * The strip draws each frame when the instant it names comes round, and on
     * this board that instant has to be converted out of master time first.
     *
     * Read live rather than captured: stream_offset is slewed toward the
     * estimate at 200 ppm, so a copy taken once would go stale at exactly the
     * crystal difference -- which is the bug docs/clock-sync.md section 9
     * records in the audio path, where the servo was fed its own drift as its
     * reference and faithfully parked the speaker at the growing error.
     */
    visualiser_set_clock(vis_master_to_local);
#else
    ESP_LOGW(TAG, "visualiser DISABLED (menuconfig) -- LEDs will stay dark");
#endif

    xTaskCreate(probe_task, "probe", 4096, NULL, 6, NULL);
    xTaskCreate(rx_task, "rx", 4096, NULL, 7, NULL);
    xTaskCreatePinnedToCore(play_task, "play", 4096, NULL, 8, NULL, 1);
    xTaskCreate(drift_task, "drift", 3072, NULL, 3, NULL);

    log_build_stamp(TAG);
    ESP_LOGI(TAG, "satellite up, joining \"%s\"", AP_SSID);
}
