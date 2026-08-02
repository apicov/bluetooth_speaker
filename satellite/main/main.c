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
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/i2s_std.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
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
static i2s_chan_handle_t i2s_tx;

/* Local-clock instant the next byte entering the ring should reach the DAC.
 * Zero means playback has not started. */
static volatile int64_t stream_start_local;
static volatile uint32_t stream_rate = 44100;

/* ------------------------------------------------------------------- wifi */

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
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_connect());
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
        /* First chunk of a stream: this is the only moment playback timing is
         * decided. Everything after is carried by I2S clocking itself. */
        stream_rate = msg->sample_rate ? msg->sample_rate : 44100;
        sbc_decoder_init();
        stream_start_local = sync_to_local(msg->play_at, offset);
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
        if (xStreamBufferSend(ring, pcm, samples * sizeof(int16_t), 0)
                != samples * sizeof(int16_t)) {
            ESP_LOGW(TAG, "ring full, dropping audio");
        }
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
        ESP_LOGI(TAG, "playback started");

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
            size_t written = 0;
            i2s_channel_write(i2s_tx, chunk, sizeof(chunk), &written, portMAX_DELAY);
        }
    }
}

/*
 * The ring's fill level is the drift signal. With both boards nominally at
 * 44.1 kHz but ~14 ppm apart, a satellite clocking slightly slow accumulates
 * audio it cannot consume, and one clocking fast drains toward underrun.
 * M6 will act on this number; for now it just has to be visible.
 */
static void monitor_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (stream_start_local != 0) {
            size_t filled = RING_BYTES - xStreamBufferSpacesAvailable(ring);
            ESP_LOGI(TAG, "buffer %u bytes (%lu ms)",
                     (unsigned)filled,
                     (unsigned long)(filled * 1000 / (stream_rate * AUDIO_CHANNELS * 2)));
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

    xTaskCreate(probe_task, "probe", 4096, NULL, 6, NULL);
    xTaskCreate(rx_task, "rx", 4096, NULL, 7, NULL);
    xTaskCreatePinnedToCore(play_task, "play", 4096, NULL, 8, NULL, 1);
    xTaskCreate(monitor_task, "mon", 3072, NULL, 3, NULL);

    log_build_stamp(TAG);
    ESP_LOGI(TAG, "satellite up, joining \"%s\"", AP_SSID);
}
