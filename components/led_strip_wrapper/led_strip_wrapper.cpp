#include "led_strip_wrapper.hpp"
#include "led_strip.h"
#include "led_strip_rmt.h"
#include "led_strip_spi.h"
#include "esp_log.h"

static const char *TAG = "led_strip";

// ─── GRBW format_id for SK6812 ────────────────────────────────────────────────
//
// led_color_component_format_t is a union of a bitfield struct and uint32_t.
// We set format_id directly to avoid C compound literals, which are unreliable
// in C++ aggregate initialisation.
//
// Bit layout (from led_strip_types.h):
//   [1:0]  r_pos  = 1  (red is 2nd byte on the wire)
//   [3:2]  g_pos  = 0  (green is 1st byte on the wire)
//   [5:4]  b_pos  = 2  (blue is 3rd byte)
//   [7:6]  w_pos  = 3  (white is 4th byte)
//   [26:8] reserved = 0
//   [28:27] bytes_per_color = 1
//   [31:29] num_components  = 4  (RGBW has 4 channels)
//
// Resulting format_id = 0x880000E1
static constexpr uint32_t FORMAT_ID_GRBW = 0x880000E1u;

LedStrip::LedStrip(gpio_num_t pin, uint32_t num_leds, Type type, Backend backend)
    : num_leds_(num_leds), type_(type)
{
    // Zero-init both config structs so all unset fields take their
    // driver-defined defaults (clk_src=0 → APB, format_id=0 → GRB fallback).
    led_strip_config_t strip_config{};
    strip_config.strip_gpio_num = static_cast<int>(pin);
    strip_config.max_leds       = num_leds;

    if (type == Type::SK6812_RGBW) {
        strip_config.led_model = LED_MODEL_SK6812;
        strip_config.color_component_format.format_id = FORMAT_ID_GRBW;
    } else if (type == Type::WS2811) {
        strip_config.led_model = LED_MODEL_WS2811;
        // This strip is BRG wired (OUT1→Blue, OUT2→Red, OUT3→Green).
        // r_pos=1, g_pos=2, b_pos=0, w_pos=3, bytes_per_color=1, num_components=3
        //   bits[31:29]=011, bits[28:27]=01, bits[7:6]=11, bits[5:4]=00, bits[3:2]=10, bits[1:0]=01
        //   → 0x680000C9
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
