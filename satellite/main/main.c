/*
 * Dancefloor satellite -- M5.
 *
 * Joins the master's SoftAP, keeps its clock aligned with the master's, receives
 * multicast PCM chunks and plays them at the instant each chunk was stamped for.
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
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_app_desc.h"
#include "nvs_flash.h"

#include "sync_proto.h"
#include "sbc_decoder.h"

#define AP_SSID    "dancefloor"
#define AP_PASS    "dancefloor"
#define MASTER_IP  "192.168.4.1"        /* esp_netif SoftAP default */

#define PROBE_PERIOD_MS 250             /* see docs/clock-sync.md §3 */

/* Must hold the master's lead time (250 ms ~ 44 kB at 44.1 kHz stereo) plus
 * headroom for jitter. */
#define RING_BYTES  (64 * 1024)

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
static volatile uint32_t stream_rate = 44100;
static uint32_t tx_rate = 44100;      /* what the output clock is actually set to */
static int64_t stream_offset;         /* clock offset used when anchoring */
/*
 * 32-bit, not 64: these are read by the playback task while the receive task
 * writes them, and a 64-bit load is two instructions on this CPU -- a torn read
 * yields a garbage position and a wild marker. int32 holds 13 hours of frames
 * at 44.1 kHz, which is longer than any party.
 */
static volatile int32_t marker_sample = -1;   /* ring position of a tagged packet */
static volatile int32_t samples_in;           /* frames written into the ring */

/* Target buffer depth: the hub stamps audio ~200 ms ahead, so in steady state
 * that much should be sitting here waiting. */
#define RING_TARGET_MS 200

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
        ESP_LOGW(TAG, "disconnected from \"%s\" (reason %d), retrying",
                 AP_SSID, d->reason);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *e = data;
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
    ESP_LOGW(TAG, "internal DAC output: GPIO 25 = left, GPIO 26 = right (8-bit)");
}

/* int16 signed interleaved -> uint8 unsigned, which is what the DAC wants. */
static void write_audio(const uint8_t *pcm, size_t bytes)
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
}

static void write_audio(const uint8_t *pcm, size_t bytes)
{
    size_t written = 0;
    i2s_channel_write(i2s_tx, pcm, bytes, &written, portMAX_DELAY);
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
        vTaskDelay(pdMS_TO_TICKS(PROBE_PERIOD_MS));
    }
}

static void handle_audio(const audio_msg_t *msg)
{
    static uint32_t expect_seq;
    static bool have_seq;
    static int16_t pcm[SBC_MAX_PCM_SAMPLES];

    int64_t offset;
    if (!sync_est_offset(&est, &offset)) {
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
         * so it must use a settled clock estimate. Three probes are enough to
         * produce *an* offset but not a good one -- minimum-RTT selection needs
         * a full window to find a genuinely uncongested round trip, and any
         * error here is baked in for the life of the stream.
         */
        if (!sync_est_settled(&est)) {
            static bool told;
            if (!told) {
                told = true;
                ESP_LOGI(TAG, "holding playback until the clock estimate settles");
            }
            return;
        }
        stream_rate = msg->sample_rate ? msg->sample_rate : 44100;
        sbc_decoder_init();
        stream_start_local = sync_to_local(msg->play_at, offset);
        stream_offset = offset;
        samples_in = 0;
        marker_sample = -1;
        expect_seq = msg->seq;
        have_seq = true;
        xStreamBufferReset(ring);
        ESP_LOGI(TAG, "stream start: play_at %lld -> local %lld (in %lld ms)",
                 msg->play_at, stream_start_local,
                 (stream_start_local - esp_timer_get_time()) / 1000);
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
        ESP_LOGW(TAG, "lost %" PRIu32 " packet(s), inserting %" PRIu32 " frames of silence",
                 missing, frames_missing);
        while (frames_missing >= 128) {
            xStreamBufferSend(ring, silence, sizeof(silence), 0);
            frames_missing -= 128;
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

    /* Decode here rather than at the hub: that is the entire point of sending
     * SBC, and it costs a quarter of the airtime. */
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
        if (xStreamBufferSend(ring, pcm, want, 0) != want) {
            ESP_LOGW(TAG, "ring full, dropping audio");
        }
        samples_in += (int32_t)(samples / AUDIO_CHANNELS);
    }
}

static void rx_task(void *arg)
{
    (void)arg;
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
        } else if (buf[0] == MSG_AUDIO && n >= (int)AUDIO_MSG_BYTES(0)) {
            handle_audio((const audio_msg_t *)buf);
        }
    }
}

/* ----------------------------------------------------------------- drift */

#if !CONFIG_DANCEFLOOR_USE_INTERNAL_DAC
static void retune_output(uint32_t hz)
{
    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(hz);
    ESP_ERROR_CHECK(i2s_channel_disable(i2s_tx));
    ESP_ERROR_CHECK(i2s_channel_reconfig_std_clock(i2s_tx, &clk));
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx));
    ESP_LOGW(TAG, "output clock retuned %" PRIu32 " -> %" PRIu32 " Hz", tx_rate, hz);
    tx_rate = hz;
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
        if (stream_start_local == 0) {
            continue;
        }

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
        err_ema = err_ema_valid ? (err_ema * 3 + err_frames) / 4 : err_frames;
        err_ema_valid = true;

        ESP_LOGI(TAG, "buffer %u bytes (%lu ms) | drift %+ld ms (smoothed %+ld ms)",
                 (unsigned)filled,
                 (unsigned long)(filled * 1000 / (stream_rate * AUDIO_CHANNELS * 2)),
                 (long)(err_frames * 1000 / (int32_t)stream_rate),
                 (long)(err_ema * 1000 / (int32_t)stream_rate));

#if CONFIG_DANCEFLOOR_USE_INTERNAL_DAC
        /* The internal DAC's rate is fixed at creation, so there is nothing to
         * trim. Fine for bench listening, not for a unit that must stay in sync
         * for hours -- use an I2S DAC for that. */
        (void)err_frames;
#else
        uint32_t desired = (uint32_t)((int32_t)stream_rate + err_ema / 40);
        /*
         * Threshold now applies to the smoothed error, so it can be far tighter:
         * 0.02% instead of 0.15%. That is ~8 ms of accumulated drift before a
         * correction, against ~60 ms before -- comfortably inside the ~5 ms that
         * starts to matter between two speakers, while jitter no longer reaches
         * it at all.
         */
        if (cooldown > 0) {
            cooldown--;
        } else if (desired > tx_rate + tx_rate / 5000 ||
                   desired < tx_rate - tx_rate / 5000) {
            retune_output(desired);
            cooldown = 4;        /* ~20 s, against a 40 s correction time */
        }
#endif
    }
}

/* --------------------------------------------------------------- playback */

static void play_task(void *arg)
{
    (void)arg;
    static uint8_t chunk[AUDIO_CHUNK_BYTES];

    while (1) {
        if (stream_start_local == 0) {
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

        while (1) {
            size_t got = xStreamBufferReceive(ring, chunk, sizeof(chunk), pdMS_TO_TICKS(500));
            if (got == 0) {
                ESP_LOGW(TAG, "underrun, waiting for a new stream");
                stream_start_local = 0;
                break;
            }
            if (got < sizeof(chunk)) {
                memset(chunk + got, 0, sizeof(chunk) - got);
            }
            int32_t mark = marker_sample;
            if (mark >= 0 && samples_played >= mark) {
                gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 1);
                esp_rom_delay_us(MARKER_PULSE_US);
                gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 0);
                marker_sample = -1;
            }
            samples_played += AUDIO_FRAMES;

            write_audio(chunk, sizeof(chunk));
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

    sync_est_init(&est);
    /* Link check only for now -- the SBC receive path lands in the next stage. */
    ESP_LOGI(TAG, "SBC decoder init: %s", sbc_decoder_init() ? "ok" : "FAILED");
    ring = xStreamBufferCreate(RING_BYTES, AUDIO_CHUNK_BYTES);
    assert(ring);

    wifi_start_sta();
    socket_start();
    i2s_start(44100);
    tx_rate = 44100;

    gpio_config_t marker = {
        .pin_bit_mask = 1ULL << CONFIG_DANCEFLOOR_MARKER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&marker));
    ESP_LOGI(TAG, "sync marker on GPIO %d", CONFIG_DANCEFLOOR_MARKER_GPIO);

    xTaskCreate(probe_task, "probe", 4096, NULL, 6, NULL);
    xTaskCreate(rx_task, "rx", 4096, NULL, 7, NULL);
    xTaskCreatePinnedToCore(play_task, "play", 4096, NULL, 8, NULL, 1);
    xTaskCreate(drift_task, "drift", 3072, NULL, 3, NULL);

    log_build_stamp(TAG);
    ESP_LOGI(TAG, "satellite up, joining \"%s\"", AP_SSID);
}
