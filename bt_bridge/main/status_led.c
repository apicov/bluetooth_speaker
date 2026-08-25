#include "status_led.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#define PIN_CONNECTED CONFIG_BRIDGE_LED_CONNECTED_GPIO
#define PIN_STREAMING CONFIG_BRIDGE_LED_STREAMING_GPIO

#define HAVE_CONNECTED (PIN_CONNECTED >= 0)
#define HAVE_STREAMING (PIN_STREAMING >= 0)

#if CONFIG_BRIDGE_LED_ACTIVE_LOW
#define LED_ON  0
#define LED_OFF 1
#else
#define LED_ON  1
#define LED_OFF 0
#endif

#define TICK_MS 100
#define BLINK_HALF_MS 1000

#define SILENCE_US 300000

static volatile bool s_connected;
static volatile int64_t s_last_audio_us;

void status_led_set_connected(bool connected)
{
    s_connected = connected;
}

void status_led_note_audio(void)
{
    s_last_audio_us = esp_timer_get_time();
}

static const char *TAG = "status_led";

static void led_task(void *arg)
{
    bool blink = false;
    bool was_streaming = false;
    int since_edge_ms = 0;
    int64_t edge_us = 0;

    while (1) {
#if HAVE_CONNECTED
        gpio_set_level(PIN_CONNECTED, s_connected ? LED_ON : LED_OFF);
#endif

        const int64_t now_us = esp_timer_get_time();
        const bool streaming =
            s_last_audio_us != 0 && (now_us - s_last_audio_us) < SILENCE_US;

        if (streaming != was_streaming && s_last_audio_us != 0) {
            const int64_t held = edge_us ? now_us - edge_us : 0;
            if (streaming) {
                ESP_LOGW(TAG, "audio resumed after %lld ms of silence",
                         held / 1000);
            } else {

                ESP_LOGW(TAG, "audio stopped, last packet %lld ms ago",
                         (now_us - s_last_audio_us) / 1000);
            }
            edge_us = now_us;
        }

        if (!streaming) {

            blink = false;
            since_edge_ms = 0;
        } else if (!was_streaming) {

            blink = true;
            since_edge_ms = 0;
        } else if ((since_edge_ms += TICK_MS) >= BLINK_HALF_MS) {
            blink = !blink;
            since_edge_ms = 0;
        }
        was_streaming = streaming;

#if HAVE_STREAMING
        gpio_set_level(PIN_STREAMING, blink ? LED_ON : LED_OFF);
#endif
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    }
}

void status_led_start(void)
{
#if HAVE_CONNECTED || HAVE_STREAMING

    const gpio_config_t cfg = {
        .pin_bit_mask =
#if HAVE_CONNECTED
            (1ULL << PIN_CONNECTED) |
#endif
#if HAVE_STREAMING
            (1ULL << PIN_STREAMING) |
#endif
            0,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));

#if HAVE_CONNECTED
    gpio_set_level(PIN_CONNECTED, LED_OFF);
#endif
#if HAVE_STREAMING
    gpio_set_level(PIN_STREAMING, LED_OFF);
#endif
#endif

    if (xTaskCreate(led_task, "status_led", 2048, NULL, 1, NULL) != pdPASS) {

        ESP_LOGE(TAG, "TASK \"status_led\" FAILED TO START -- the front panel "
                      "will stay dark and does NOT mean the link is down");
    }

    ESP_LOGI(TAG, "status LEDs: connected on %d, streaming on %d (-1 = none), "
                  "active %s; audio gaps logged either way",
             PIN_CONNECTED, PIN_STREAMING, LED_ON == 0 ? "low" : "high");
}
