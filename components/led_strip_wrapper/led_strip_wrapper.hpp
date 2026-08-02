#pragma once
#include <cstdint>
#include "driver/gpio.h"
#include "esp_err.h"
#include "led_strip.h"

/**
 * @brief Thin RAII wrapper around the ESP-IDF led_strip driver.
 *
 * Supports both 3-channel (WS2812, GRB) and 4-channel (SK6812, GRBW) strips.
 * Pixel data is staged in RAM and only pushed to the hardware when show() is called.
 */
class LedStrip {
public:
    /// Physical strip variant — controls timing and byte order.
    enum class Type {
        WS2812,      ///< 3-channel, GRB wire order, 5V
        WS2811,      ///< 3-channel, RGB wire order, 12V (1 IC drives 3 physical LEDs)
        SK6812_RGBW, ///< 4-channel, GRBW wire order, 5V
    };

    /**
     * @brief Which peripheral clocks the data line out.
     *
     * RMT is the natural fit and the default. It does not survive a heavy
     * continuous workload on the original ESP32, though: led_strip's RMT backend
     * calls rmt_enable() / rmt_disable() around every single frame, and
     * rmt_disable races the transmit-done ISR as it walks the channel through
     * RUN -> WAIT -> ENABLE. Landing in WAIT fails with "channel can't be
     * disabled in state 3", after which the channel is stuck enabled and every
     * later frame fails. Measured here at roughly 650 frames to first failure
     * with a busy radio stack sharing the chip.
     *
     * SPI transmits in one DMA-driven transfer with no enable/disable cycle, so
     * there is no state machine to race. It costs a whole SPI bus to use one
     * MOSI pin. Prefer it anywhere the strip refreshes continuously.
     */
    enum class Backend {
        RMT,   ///< Default. Fine for intermittent updates.
        SPI,   ///< DMA, no per-frame enable/disable. Reserves an SPI bus.
    };

    /**
     * @brief Initialise the driving peripheral and LED strip.
     * @param pin      GPIO driving the data line.
     * @param num_leds Number of LEDs in the strip.
     * @param type     Strip model (default: WS2812).
     * @param backend  Peripheral to drive it with (default: RMT).
     */
    LedStrip(gpio_num_t pin, uint32_t num_leds, Type type = Type::WS2812,
             Backend backend = Backend::RMT);
    ~LedStrip();

    /// Not copyable: the handle is owned, and a double free would follow.
    LedStrip(const LedStrip &) = delete;
    LedStrip &operator=(const LedStrip &) = delete;

    /**
     * @brief Stage an RGB pixel (white channel set to 0 for RGBW strips).
     * @param index Zero-based LED index.
     * @param r,g,b Colour components (0–255).
     */
    void set(uint32_t index, uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Stage an RGBW pixel (SK6812 strips only; works on WS2812 too, w ignored).
     * @param index   Zero-based LED index.
     * @param r,g,b,w Colour components (0–255).
     */
    void set(uint32_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w);

    /// Set all pixels to off.
    void clear();

    /**
     * @brief Transmit staged pixel data to the hardware.
     * @return ESP_OK, or the driver's error.
     *
     * Returns rather than asserts: a wedged strip fails on every frame, and a
     * caller rendering continuously wants to log the first failure and carry on
     * rather than abort or flood the console.
     */
    esp_err_t show();

private:
    led_strip_handle_t handle_;
    uint32_t           num_leds_;
    Type               type_;
};
