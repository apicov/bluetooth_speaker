/*
 * I2S link integrity test -- no DAC, no Bluetooth, no phone.
 *
 * Roles are chosen in menuconfig:
 *
 *   TX_ONLY  -> flash to the BRIDGE board. Replaces A2DP audio with a known
 *               counting pattern on the same I2S pins.
 *   RX_ONLY  -> flash to the HUB board. Verifies what arrives.
 *   BOTH     -> one board, looped back to itself, which isolates the driver
 *               from the wiring.
 *
 * Across two boards the existing four wires are used unchanged:
 *
 *     bridge GPIO 26 (BCK)  ->  hub GPIO 21
 *     bridge GPIO 27 (WS)   ->  hub GPIO 22
 *     bridge GPIO 25 (DATA) ->  hub GPIO 23
 *     GND                   ->  GND
 *
 * TX is master and RX is slave, as the real system is arranged.
 *
 * The transmitted pattern is a 16-bit counter, identical in both channels. Every
 * received frame must be exactly one more than the last. Anything else is a lost
 * frame, and the size of the jump says how many -- no rate arithmetic, no
 * assumptions, just counting.
 *
 * PCNT on the WS line gives an independent hardware frame count at the pin, so
 * losses inside the driver can be told apart from frames that never arrived.
 */
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#define RATE            44100
#define FRAMES_PER_BUF  1024                 /* ~23 ms, matching the hub */
#define BUF_SAMPLES     (FRAMES_PER_BUF * 2) /* stereo */

#define TX_BCK 26
#define TX_WS  27
#define TX_DO  25
#define RX_BCK 21
#define RX_WS  22
#define RX_DI  23

#define PCNT_HIGH 20000

static const char *TAG = "loop";

static i2s_chan_handle_t tx_chan, rx_chan;
static pcnt_unit_handle_t pcnt_unit;
static volatile int64_t ws_accum;

/* --------------------------------------------------------------- WS counter */

#if !CONFIG_DANCEFLOOR_LOOPBACK_TX_ONLY
static bool IRAM_ATTR on_pcnt_reach(pcnt_unit_handle_t u,
                                    const pcnt_watch_event_data_t *ev, void *ctx)
{
    ws_accum += PCNT_HIGH;
    return false;
}

static int64_t ws_total(void)
{
    int c = 0;
    pcnt_unit_get_count(pcnt_unit, &c);
    return ws_accum + c;
}

static void ws_counter_start(void)
{
    pcnt_unit_config_t ucfg = { .high_limit = PCNT_HIGH, .low_limit = -1 };
    ESP_ERROR_CHECK(pcnt_new_unit(&ucfg, &pcnt_unit));
    pcnt_chan_config_t ccfg = { .edge_gpio_num = RX_WS, .level_gpio_num = -1 };
    pcnt_channel_handle_t ch = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &ccfg, &ch));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, PCNT_HIGH));
    pcnt_event_callbacks_t cbs = { .on_reach = on_pcnt_reach };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, NULL));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}
#endif

/* -------------------------------------------------------------------- i2s */

static void i2s_setup(void)
{
#if CONFIG_DANCEFLOOR_LOOPBACK_RX_IS_MASTER
#  define TX_ROLE I2S_ROLE_SLAVE
#  define RX_ROLE I2S_ROLE_MASTER
#else
#  define TX_ROLE I2S_ROLE_MASTER
#  define RX_ROLE I2S_ROLE_SLAVE
#endif

#if !CONFIG_DANCEFLOOR_LOOPBACK_RX_ONLY
    i2s_chan_config_t tx_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, TX_ROLE);
    tx_cfg.dma_desc_num = 8;
    tx_cfg.dma_frame_num = 512;
    ESP_ERROR_CHECK(i2s_new_channel(&tx_cfg, &tx_chan, NULL));

    i2s_std_config_t tx_std = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .bclk = TX_BCK, .ws = TX_WS,
                      .dout = TX_DO, .din = I2S_GPIO_UNUSED,
                      /* Only meaningful when this side is the slave. */
                      .invert_flags = { .bclk_inv = CONFIG_DANCEFLOOR_LOOPBACK_SLAVE_BCLK_INV } },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &tx_std));
#endif

#if !CONFIG_DANCEFLOOR_LOOPBACK_TX_ONLY
    i2s_chan_config_t rx_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, RX_ROLE);
    rx_cfg.dma_desc_num = 8;
    rx_cfg.dma_frame_num = 512;
    ESP_ERROR_CHECK(i2s_new_channel(&rx_cfg, NULL, &rx_chan));

    i2s_std_config_t rx_std = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .bclk = RX_BCK, .ws = RX_WS,
                      .dout = I2S_GPIO_UNUSED, .din = RX_DI },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &rx_std));
#if !CONFIG_DANCEFLOOR_LOOPBACK_RX_IS_MASTER
    /* Slave first, so it is listening before any clock starts. */
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
#endif
#endif

#if !CONFIG_DANCEFLOOR_LOOPBACK_TX_ONLY && CONFIG_DANCEFLOOR_LOOPBACK_RX_IS_MASTER
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));   /* clock source last */
#endif

#if !CONFIG_DANCEFLOOR_LOOPBACK_RX_ONLY
    /* After i2s_channel_init_std_mode() has claimed the pins, so this overrides
     * the driver's default rather than being overwritten by it. */
    const gpio_drive_cap_t cap = (gpio_drive_cap_t)CONFIG_DANCEFLOOR_LOOPBACK_DRIVE;
    ESP_ERROR_CHECK(gpio_set_drive_capability(TX_BCK, cap));
    ESP_ERROR_CHECK(gpio_set_drive_capability(TX_WS, cap));
    ESP_ERROR_CHECK(gpio_set_drive_capability(TX_DO, cap));
    ESP_LOGW(TAG, "TX drive capability %d", cap);

    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
#endif
}

/* -------------------------------------------------------------------- tasks */

#if !CONFIG_DANCEFLOOR_LOOPBACK_RX_ONLY
static void tx_task(void *arg)
{
    static int16_t buf[BUF_SAMPLES];
    uint16_t counter = 0;

    while (1) {
        for (int i = 0; i < FRAMES_PER_BUF; i++) {
            buf[2 * i]     = (int16_t)counter;   /* same value in both channels, so a */
            buf[2 * i + 1] = (int16_t)counter;   /* channel swap is visible too */
            counter++;
        }
        size_t written = 0;
        i2s_channel_write(tx_chan, buf, sizeof(buf), &written, portMAX_DELAY);
    }
}
#endif

#if !CONFIG_DANCEFLOOR_LOOPBACK_TX_ONLY
static void rx_task(void *arg)
{
    static int16_t buf[BUF_SAMPLES];
    bool primed = false;
    uint16_t expect = 0;

    int64_t t0 = 0, ws0 = 0;
    uint64_t frames = 0, lost = 0;
    uint32_t breaks = 0, chan_swaps = 0;

    while (1) {
        size_t got = 0;
        if (i2s_channel_read(rx_chan, buf, sizeof(buf), &got, portMAX_DELAY) != ESP_OK) {
            continue;
        }
        int n = got / 4;

        for (int i = 0; i < n; i++) {
            uint16_t l = (uint16_t)buf[2 * i];
            uint16_t r = (uint16_t)buf[2 * i + 1];
            if (l != r) {
                chan_swaps++;             /* left and right disagree -> frame slip */
            }
            if (!primed) {
                primed = true;
            } else if (l != expect) {
                /* Unsigned difference gives the gap size directly, wraparound
                 * included. A huge value means a slip, not a drop. */
                lost += (uint16_t)(l - expect);
                breaks++;
            }
            expect = l + 1;
        }
        frames += n;

        int64_t now = esp_timer_get_time();
        if (t0 == 0) {
            t0 = now; ws0 = ws_total(); frames = 0; lost = 0; breaks = 0; chan_swaps = 0;
            continue;
        }
        if (now - t0 >= 5000000) {
            int64_t ws = ws_total();
            ESP_LOGI(TAG, "rx %llu f/s | pin %lld f/s | lost %llu (%u breaks) | swaps %u",
                     frames * 1000000 / (uint64_t)(now - t0),
                     (ws - ws0) * 1000000 / (now - t0),
                     lost, breaks, chan_swaps);
            t0 = now; ws0 = ws; frames = 0; lost = 0; breaks = 0; chan_swaps = 0;
        }
    }
}
#endif

void app_main(void)
{
#if CONFIG_DANCEFLOOR_LOOPBACK_TX_ONLY
    ESP_LOGW(TAG, "role TX (%s) -- BCK %d / WS %d / DATA %d",
             CONFIG_DANCEFLOOR_LOOPBACK_RX_IS_MASTER ? "I2S slave" : "I2S MASTER",
             TX_BCK, TX_WS, TX_DO);
#elif CONFIG_DANCEFLOOR_LOOPBACK_RX_ONLY
    ESP_LOGW(TAG, "role RX (%s) -- BCK %d / WS %d / DIN %d",
             CONFIG_DANCEFLOOR_LOOPBACK_RX_IS_MASTER ? "I2S MASTER" : "I2S slave",
             RX_BCK, RX_WS, RX_DI);
#else
    ESP_LOGW(TAG, "role BOTH -- self-loopback, wire %d->%d, %d->%d, %d->%d",
             TX_BCK, RX_BCK, TX_WS, RX_WS, TX_DO, RX_DI);
#endif

    i2s_setup();

#if !CONFIG_DANCEFLOOR_LOOPBACK_TX_ONLY
    ws_counter_start();
    xTaskCreatePinnedToCore(rx_task, "rx", 4096, NULL, 9, NULL, 1);
#endif
#if !CONFIG_DANCEFLOOR_LOOPBACK_RX_ONLY
    xTaskCreatePinnedToCore(tx_task, "tx", 4096, NULL, 8, NULL, 1);
#endif
}
