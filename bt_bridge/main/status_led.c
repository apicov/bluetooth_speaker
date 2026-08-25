/**
 * @file status_led.c
 * @brief Drives the two front-panel LEDs from one task, and logs the audio
 *        gaps that the blinking one is about. Declared in status_led.h.
 *
 * The task is the only thing that touches either pin. Connection state could be
 * written straight from the A2DP callback, but the blink cannot -- it needs a
 * clock -- and one owner for both means they can never disagree about who last
 * wrote a pin. It also keeps gpio_set_level() out of the Bluetooth stack's
 * callback context, which is the one place on this chip with no time to spare.
 */
#include "status_led.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

/** @brief Pin for the LED that is solid while a phone is connected. */
#define PIN_CONNECTED CONFIG_BRIDGE_LED_CONNECTED_GPIO
/** @brief Pin for the LED that blinks while audio is moving. */
#define PIN_STREAMING CONFIG_BRIDGE_LED_STREAMING_GPIO

/**
 * @brief Whether the connected LED exists at all; a negative pin removes it.
 *
 * Tested with `#if` rather than `if`, because a disabled pin would otherwise
 * leave a shift by a negative count in the source for the compiler to fold and
 * complain about, dead branch or not.
 */
#define HAVE_CONNECTED (PIN_CONNECTED >= 0)
/** @brief The same for the streaming LED. @see HAVE_CONNECTED */
#define HAVE_STREAMING (PIN_STREAMING >= 0)

#if CONFIG_BRIDGE_LED_ACTIVE_LOW
/**
 * @brief The level that lights an LED, which is a property of the wiring and
 *        not of anything this file decides.
 *
 * With the anodes on 3V3 and the cathodes on the pins, the pin sinks the
 * current and a LOW level lights it; wired the other way round the sense
 * inverts, and CONFIG_BRIDGE_LED_ACTIVE_LOW selects between the two. Everything
 * below is written as #LED_ON and #LED_OFF rather than as 1 and 0, so the logic
 * reads the same either way and a rewired board is a menuconfig change rather
 * than a patch.
 *
 * Getting it wrong does not produce a dark LED, which is the trap. The
 * connected LED reads BACKWARDS -- solid whenever no phone is connected -- and
 * the streaming LED blinks in antiphase, which is undetectable on its own.
 */
#define LED_ON  0
/** @brief The level that darkens an LED. @see LED_ON */
#define LED_OFF 1
#else
#define LED_ON  1
#define LED_OFF 0
#endif

/**
 * @brief The task period, and what decides how quickly either LED reacts to a
 *        phone connecting or a stream stopping.
 *
 * Fast because the connected LED and the silence check both want it, and
 * independent of the blink rate below.
 */
#define TICK_MS 100
/**
 * @brief Half the blink period: one edge every this many milliseconds, so on
 *        for a second and off for a second.
 */
#define BLINK_HALF_MS 1000

/**
 * @brief How long after the last packet the streaming LED goes dark.
 *
 * A2DP delivers packets continuously while playing, many per second, so a gap
 * of this length means the phone paused, went away, or the stream broke. Long
 * enough that ordinary jitter never blinks the LED off, and short enough that a
 * pause reads as immediate.
 */
#define SILENCE_US 300000

/** @brief Last state given to status_led_set_connected(). */
static volatile bool s_connected;
/** @brief When status_led_note_audio() last ran, in esp_timer microseconds;
 *         zero until the first packet. */
static volatile int64_t s_last_audio_us;

/* Declared in status_led.h, like the one below it. */
void status_led_set_connected(bool connected)
{
    s_connected = connected;
}

void status_led_note_audio(void)
{
    s_last_audio_us = esp_timer_get_time();
}

/** @brief ESP_LOG tag for the panel and the audio-gap lines. */
static const char *TAG = "status_led";

/**
 * @brief Drive both LEDs and report the two transitions of the audio stream.
 * @param arg  Unused.
 */
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
        /*
         * Deliberately not under #if HAVE_STREAMING, though the LED it also
         * drives is. This test is the only thing on this chip that knows the
         * A2DP source has gone quiet: when the phone stops feeding, nothing
         * else here logs anything at all, so the hub starves while the bridge's
         * console shows a hole, and a phone that stopped sending cannot be told
         * from a bridge that failed to forward.
         *
         * A diagnostic that disappears when someone does not wire an LED is
         * useless on exactly the rig that needs it, so only the
         * gpio_set_level() below is conditional.
         */
        const int64_t now_us = esp_timer_get_time();
        const bool streaming =
            s_last_audio_us != 0 && (now_us - s_last_audio_us) < SILENCE_US;

        /* Edges only. The steady states are the LED's business, and a line per
         * tick would bury the two transitions that matter. */
        if (streaming != was_streaming && s_last_audio_us != 0) {
            const int64_t held = edge_us ? now_us - edge_us : 0;
            if (streaming) {
                ESP_LOGW(TAG, "audio resumed after %lld ms of silence",
                         held / 1000);
            } else {
                /* Dated from the last packet, not from this tick: the silence
                 * began when the audio did, and SILENCE_US plus up to a tick of
                 * latency is this task noticing, not the gap itself. */
                ESP_LOGW(TAG, "audio stopped, last packet %lld ms ago",
                         (now_us - s_last_audio_us) / 1000);
            }
            edge_us = now_us;
        }

        if (!streaming) {
            /* Settle dark rather than freezing wherever the last edge left it:
             * an LED stopped mid-blink and one lit solid read the same. */
            blink = false;
            since_edge_ms = 0;
        } else if (!was_streaming) {
            /* Light the moment audio starts, rather than up to a half period
             * later -- at this rate, waiting one out would look like the LED
             * had missed the stream entirely. */
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

/* Declared in status_led.h. */
void status_led_start(void)
{
#if HAVE_CONNECTED || HAVE_STREAMING
    /* Pins only. The task below starts either way -- it carries the audio-gap
     * diagnostic, which a board with no LEDs still wants. */
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
     * solid at boot and stay that way until the first tick. It is only one tick,
     * but "both LEDs solid" is what a hung bridge looks like, and the boot is
     * exactly when someone is watching for that. */
#if HAVE_CONNECTED
    gpio_set_level(PIN_CONNECTED, LED_OFF);
#endif
#if HAVE_STREAMING
    gpio_set_level(PIN_STREAMING, LED_OFF);
#endif
#endif

    /* Lowest priority in the system: a missed tick shows as a slightly uneven
     * blink and nothing else, which is the right thing to give up first. */
    if (xTaskCreate(led_task, "status_led", 2048, NULL, 1, NULL) != pdPASS) {
        /* Checked like the rest, though this is the one whose loss costs
         * nothing but the panel: dark LEDs would otherwise read as "no phone
         * connected", which is a wrong answer rather than a missing one. */
        ESP_LOGE(TAG, "TASK \"status_led\" FAILED TO START -- the front panel "
                      "will stay dark and does NOT mean the link is down");
    }

    ESP_LOGI(TAG, "status LEDs: connected on %d, streaming on %d (-1 = none), "
                  "active %s; audio gaps logged either way",
             PIN_CONNECTED, PIN_STREAMING, LED_ON == 0 ? "low" : "high");
}
