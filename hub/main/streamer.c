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

/* Held across a DAC retune. i2s_channel_write() returns immediately once the
 * channel is disabled, so without this the play task spins through the ring at
 * memory speed -- a measured 54 ms correction cost 177 ms of buffer. */
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
    /* Straight to the local ring. There is no longer an intermediate PCM buffer:
     * satellites receive SBC, so nothing needs PCM except this speaker. */
    if (!local_ring || local_start == 0) {
        return;
    }
    size_t sent = xStreamBufferSend(local_ring, pcm, len, 0);   /* must not block */
    if (sent < len) {
        s_feed_dropped += len - sent;
    }
}

void streamer_send_sbc(const uint8_t *sbc, uint16_t len, uint32_t frames)
{
    static audio_msg_t msg;
    static uint32_t seq;
    static int64_t next_play_at;

    if (sock < 0 || len == 0 || len > AUDIO_MAX_PAYLOAD || frames == 0) {
        return;
    }

    int64_t now = esp_timer_get_time();
    int64_t target = now + LEAD_US;

    if (next_play_at == 0) {
        next_play_at = target;
        local_start = target;              /* our own speaker joins the timeline */
        xStreamBufferReset(local_ring);
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

static void wifi_start_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

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
     * TX power is left at the driver default (full). The PHY rate is pinned --
     * see below, it cannot be left to the driver.
     *
     * They were previously capped at 13 dBm and pinned to 24 Mbps, to stop this
     * radio swamping the Bluetooth receiver when both shared one chip. The
     * two-chip split removed that need, and the settings then did real harm:
     * multicast frames are never acknowledged, so at a fixed 24 Mbps -- a rate
     * needing good SNR -- 23% of audio packets were lost outright, audible as
     * constant dropouts. Rate adaptation picks something the link can carry, and
     * full power gives it the margin to do so.
     *
     * The channel stays pinned: it costs nothing and helps Bluetooth's adaptive
     * frequency hopping route around us.
     */
#if CONFIG_DANCEFLOOR_WIFI_PHY_RATE_MBPS > 0
    /*
     * Multicast is never acknowledged and never retried, and left alone it goes
     * out at the 1 Mbps basic rate -- where a 1041-byte frame costs 8.3 ms and
     * 172 packets/s would need 1.43 s of airtime per second. It physically
     * cannot fit, and half the audio is dropped in the queue. Pinning the rate
     * is required for throughput, not just to be polite to Bluetooth.
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
        ESP_LOGI(TAG, "multicast pinned to %d Mbps", CONFIG_DANCEFLOOR_WIFI_PHY_RATE_MBPS);
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

static volatile int64_t s_marker_at;      /* local time we last pulsed */
static QueueHandle_t s_edge_q;            /* satellite edge timestamps */

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

    ESP_LOGI(TAG, "sync markers on GPIO %d, watching GPIO %d",
             CONFIG_DANCEFLOOR_MARKER_GPIO, CONFIG_DANCEFLOOR_MONITOR_GPIO);
}

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
}

static void retune_dac(uint32_t hz)
{
    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(hz);
    retuning = true;
    vTaskDelay(pdMS_TO_TICKS(2));        /* let the play task notice and park */
    ESP_ERROR_CHECK(i2s_channel_disable(i2s_tx));
    ESP_ERROR_CHECK(i2s_channel_reconfig_std_clock(i2s_tx, &clk));
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx));
    retuning = false;
    ESP_LOGW(TAG, "DAC clock retuned %" PRIu32 " -> %" PRIu32 " Hz", tx_rate, hz);
    tx_rate = hz;
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
        ESP_LOGI(TAG, "local playback started");
        int64_t samples_played = 0;
        int64_t next_marker = (local_start / MARKER_PERIOD_US + 1) * MARKER_PERIOD_US;

        while (1) {
            if (retuning) {
                /* Do not pull from the ring while the channel is down -- writes
                 * would return instantly and drain it. */
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
            size_t got = xStreamBufferReceive(local_ring, chunk, sizeof(chunk), pdMS_TO_TICKS(500));
            if (got == 0) {
                ESP_LOGW(TAG, "local underrun, awaiting new stream");
                local_start = 0;
                break;
            }
            if (got < sizeof(chunk)) {
                memset(chunk + got, 0, sizeof(chunk) - got);
            }
            /* On the hub, local time IS master time. */
            int64_t chunk_master = local_start + samples_played * 1000000LL / (int64_t)sample_rate;
            if (chunk_master >= next_marker) {
                gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 1);
                s_marker_at = esp_timer_get_time();
                esp_rom_delay_us(MARKER_PULSE_US);
                gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 0);
                next_marker = (chunk_master / MARKER_PERIOD_US + 1) * MARKER_PERIOD_US;
            }
            samples_played += AUDIO_FRAMES;

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
        if (local_start == 0 || rate_ema == 0) {
            continue;
        }

        size_t filled = LOCAL_RING_BYTES - xStreamBufferSpacesAvailable(local_ring);
        ESP_LOGI(TAG, "local ring %u bytes (%lu ms) | tx-fail %" PRIu32 "/5s",
                 (unsigned)filled,
                 (unsigned long)(filled * 1000 / (sample_rate * AUDIO_CHANNELS * 2)),
                 s_tx_fail);
        s_tx_fail = 0;

        const int32_t target = (int32_t)(LEAD_US / 1000) *
                               (int32_t)(rate_ema * AUDIO_CHANNELS * 2 / 1000);
        int32_t err_frames = ((int32_t)filled - target) / (AUDIO_CHANNELS * 2);
        uint32_t desired = (uint32_t)((int32_t)rate_ema + err_frames / 40);

        /* Only act on a meaningful error: every retune is an audible click. */
        if (desired > tx_rate + tx_rate / 666 || desired < tx_rate - tx_rate / 666) {
            ESP_LOGI(TAG, "servo: ring err %+ld ms -> DAC %" PRIu32 " Hz",
                     (long)(err_frames * 1000 / (int32_t)rate_ema), desired);
            retune_dac(desired);
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
    marker_start();

    ESP_LOGI(TAG, "free heap after WiFi init: %" PRIu32 " bytes", esp_get_free_heap_size());

    xTaskCreate(probe_task, "probe", 4096, NULL, 6, NULL);
    xTaskCreatePinnedToCore(local_play_task, "play", 4096, NULL, 8, NULL, 1);
    xTaskCreate(ring_monitor_task, "ringmon", 3072, NULL, 3, NULL);
    ESP_LOGI(TAG, "streaming to %s:%d", SYNC_MCAST_ADDR, SYNC_PORT);
}
