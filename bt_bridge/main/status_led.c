#include "status_led.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

/*
 * Two LEDs on the front of the bridge, so the box says what it is doing with no
 * console attached: one solid when a phone is connected, one blinking while
 * audio is actually moving.
 *
 * Both are read-only indicators -- nothing here feeds back into the link or the
 * Bluetooth stack.
 */
#define PIN_CONNECTED CONFIG_BRIDGE_LED_CONNECTED_GPIO
#define PIN_STREAMING CONFIG_BRIDGE_LED_STREAMING_GPIO

/*
 * Either pin set to -1 removes that LED. The tests are #if rather than if,
 * because a disabled pin would otherwise leave `1ULL << -1` in the source for
 * the compiler to fold and complain about, dead branch or not.
 */
#define HAVE_CONNECTED (PIN_CONNECTED >= 0)
#define HAVE_STREAMING (PIN_STREAMING >= 0)

/*
 * Which level lights an LED, which is a property of the wiring and not of
 * anything this file decides.
 *
 * Both LEDs have their anodes on 3V3 and their cathodes on the pins, so the pin
 * sinks the current and a LOW level lights it. Everything below is written as
 * LED_ON and LED_OFF rather than 1 and 0, so the logic reads the same either
 * way and a rewired board is a menuconfig change rather than a patch.
 *
 * Getting it wrong does not produce a dark LED, which is the trap. The
 * connected LED reads BACKWARDS -- solid whenever no phone is connected -- and
 * the streaming LED blinks in antiphase, which is undetectable on its own.
 */
#if CONFIG_BRIDGE_LED_ACTIVE_LOW
#define LED_ON  0
#define LED_OFF 1
#else
#define LED_ON  1
#define LED_OFF 0
#endif

/*
 * Tick period, and the blink built on top of it.
 *
 * The tick stays fast because the connected LED and the silence check below
 * both want it: it is what decides how quickly either LED reacts to a phone
 * connecting or a stream stopping. The blink rate is separate -- one edge every
 * BLINK_HALF_MS, so on for a second and off for a second is 0.5 Hz.
 */
#define TICK_MS 100
#define BLINK_HALF_MS 1000

/*
 * How long after the last packet the streaming LED goes dark.
 *
 * A2DP delivers a packet roughly every 20 ms while playing, so anything past a
 * few packets' worth of silence means the phone paused, went away, or the
 * stream broke. 300 ms is long enough that ordinary jitter never blinks the
 * LED off and short enough that a pause reads as immediate.
 */
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

#if HAVE_CONNECTED || HAVE_STREAMING
static const char *TAG = "status_led";

/*
 * The LEDs are driven from a task rather than from the events themselves.
 *
 * Connection state could be written straight to the pin from the A2DP callback,
 * but the blink cannot -- it needs a clock -- and having one owner for both
 * means the two can never disagree about who last touched a pin. It also keeps
 * gpio_set_level() out of the Bluetooth stack's callback context, which is the
 * one place on this chip with no time to spare.
 */
static void led_task(void *arg)
{
#if HAVE_STREAMING
    bool blink = false;
    bool was_streaming = false;
    int since_edge_ms = 0;
#endif

    while (1) {
#if HAVE_CONNECTED
        gpio_set_level(PIN_CONNECTED, s_connected ? LED_ON : LED_OFF);
#endif
#if HAVE_STREAMING
        const bool streaming =
            s_last_audio_us != 0 &&
            (esp_timer_get_time() - s_last_audio_us) < SILENCE_US;

        if (!streaming) {
            /* Settle dark rather than freezing wherever the last edge left it:
             * an LED stopped mid-blink and one lit solid would read the same. */
            blink = false;
            since_edge_ms = 0;
        } else if (!was_streaming) {
            /* Light the moment audio starts, rather than up to a second later.
             * The LED is slow enough now that waiting out a half period would
             * look like it had missed the stream entirely. */
            blink = true;
            since_edge_ms = 0;
        } else if ((since_edge_ms += TICK_MS) >= BLINK_HALF_MS) {
            blink = !blink;
            since_edge_ms = 0;
        }
        was_streaming = streaming;

        gpio_set_level(PIN_STREAMING, blink ? LED_ON : LED_OFF);
#endif
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    }
}
#endif

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

    /* Drive both dark before the task exists. gpio_config() leaves an output at
     * 0, which on an active-low LED is LIT -- so without this both LEDs come up
     * solid at boot and stay that way until the first tick. It is only 100 ms,
     * but "both LEDs solid" is what a hung bridge looks like, and the boot is
     * exactly when someone is watching for that. */
#if HAVE_CONNECTED
    gpio_set_level(PIN_CONNECTED, LED_OFF);
#endif
#if HAVE_STREAMING
    gpio_set_level(PIN_STREAMING, LED_OFF);
#endif

    /* Lowest priority in the system: a missed tick shows as a slightly uneven
     * blink and nothing else, which is the right thing to give up first. */
    xTaskCreate(led_task, "status_led", 2048, NULL, 1, NULL);

    ESP_LOGI(TAG, "status LEDs: connected on %d, streaming on %d (-1 = none), "
                  "active %s",
             PIN_CONNECTED, PIN_STREAMING, LED_ON == 0 ? "low" : "high");
#endif
}
