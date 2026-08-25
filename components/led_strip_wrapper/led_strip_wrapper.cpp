/**
 * @file led_strip_wrapper.cpp
 * @brief Constructing the driver handle, and the byte orders each strip wants.
 *
 * led_strip_wrapper.hpp owns the contract; what is here is the peripheral
 * setup and the wire-format constants, which is the only part of this
 * component that carries anything worth knowing.
 */
#include "led_strip_wrapper.hpp"
#include "led_strip.h"
#include "led_strip_rmt.h"
#include "led_strip_spi.h"
#include "esp_log.h"

/** @brief Log tag. */
static const char *TAG = "led_strip";

/**
 * @brief GRBW wire order for SK6812, as a packed format_id.
 *
 * led_color_component_format_t is a union of a bitfield struct and a uint32_t,
 * and format_id is set directly rather than through the fields: C compound
 * literals are unreliable in C++ aggregate initialisation, so the packed value
 * is the portable way to say this.
 *
 * Bit layout, from led_strip_types.h:
 *
 *     [1:0]   r_pos           = 1   red is the 2nd byte on the wire
 *     [3:2]   g_pos           = 0   green is the 1st
 *     [5:4]   b_pos           = 2   blue is the 3rd
 *     [7:6]   w_pos           = 3   white is the 4th
 *     [26:8]  reserved        = 0
 *     [28:27] bytes_per_color = 1
 *     [31:29] num_components  = 4   RGBW
 */
static constexpr uint32_t FORMAT_ID_GRBW = 0x880000E1u;

LedStrip::LedStrip(gpio_num_t pin, uint32_t num_leds, Type type, Backend backend)
    : num_leds_(num_leds), type_(type)
{
    // Zero-init both config structs so every unset field takes its
    // driver-defined default: clk_src 0 is APB, format_id 0 is the GRB
    // fallback, which is what a WS2812 wants.
    led_strip_config_t strip_config{};
    strip_config.strip_gpio_num = static_cast<int>(pin);
    strip_config.max_leds       = num_leds;

    if (type == Type::SK6812_RGBW) {
        strip_config.led_model = LED_MODEL_SK6812;
        strip_config.color_component_format.format_id = FORMAT_ID_GRBW;
    } else if (type == Type::WS2811) {
        strip_config.led_model = LED_MODEL_WS2811;
        // BRG as this strip is wired: OUT1 drives blue, OUT2 red, OUT3 green.
        // Packed the same way as FORMAT_ID_GRBW above -- r_pos 1, g_pos 2,
        // b_pos 0, w_pos 3, one byte per colour, three components.
        strip_config.color_component_format.format_id = 0x680000C9u; // BRG, 3-ch
    } else {
        strip_config.led_model = LED_MODEL_WS2812;
        // format_id = 0 → driver defaults to GRB (correct for WS2812)
    }

    if (backend == Backend::SPI) {
        led_strip_spi_config_t spi_config{};
        spi_config.clk_src        = SPI_CLK_SRC_DEFAULT;
        spi_config.spi_bus        = SPI2_HOST;
        spi_config.flags.with_dma = true;
        ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &handle_));
    } else {
        led_strip_rmt_config_t rmt_config{};
        rmt_config.resolution_hz = 10 * 1000 * 1000;
        // One RMT channel's memory, which differs per target: the C3 has 48
        // symbols per channel, the original ESP32 has 64. Asking for more than
        // the target has makes the driver claim a second channel.
#if CONFIG_IDF_TARGET_ESP32C3
        rmt_config.mem_block_symbols = 48;
#else
        rmt_config.mem_block_symbols = 64;
#endif
        ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &handle_));
    }

    const char* name = (type == Type::SK6812_RGBW) ? "SK6812 RGBW"
                     : (type == Type::WS2811)       ? "WS2811"
                                                    : "WS2812";
    ESP_LOGI(TAG, "%s on GPIO%d, %lu LEDs, %s backend", name, static_cast<int>(pin),
             num_leds, backend == Backend::SPI ? "SPI" : "RMT");
}

LedStrip::~LedStrip()
{
    led_strip_del(handle_);
}

void LedStrip::set(uint32_t index, uint8_t r, uint8_t g, uint8_t b)
{
    led_strip_set_pixel(handle_, index, r, g, b);
}

void LedStrip::set(uint32_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    if (type_ == Type::SK6812_RGBW)
        led_strip_set_pixel_rgbw(handle_, index, r, g, b, w);
    else
        led_strip_set_pixel(handle_, index, r, g, b);
}

void LedStrip::clear()
{
    led_strip_clear(handle_);
}

esp_err_t LedStrip::show()
{
    return led_strip_refresh(handle_);
}
