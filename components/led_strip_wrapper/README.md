# led_strip_wrapper

RAII C++ wrapper around ESP-IDF's `led_strip` driver. Vendored from the
`esp32c3_neopixel` project, with one addition: a **backend** parameter.

```cpp
LedStrip strip{GPIO_NUM_18, 60, LedStrip::Type::WS2812, LedStrip::Backend::SPI};

strip.set(0, 255, 0, 0);
strip.show();
```

## Why the backend parameter exists

The original only used RMT. That is the right default for intermittent updates and
is still what you get if you say nothing.

It does not survive continuous refresh on the original ESP32. `led_strip` 3.0.3's
RMT backend enables and disables the RMT channel around *every* frame, and
`rmt_disable` races the transmit-done ISR as it walks the channel through
RUN → WAIT → ENABLE:

```
rmt_tx_disable: channel can't be disabled in state 3
```

After that the channel is stuck enabled and every later frame fails — the strip
simply goes dark and stays dark. Measured here at roughly 650 frames with a busy
radio stack sharing the chip.

`Backend::SPI` transmits in one DMA-driven transfer with no enable/disable cycle,
so there is no state machine to race. It costs a whole SPI bus (SPI2_HOST) to use
one MOSI pin, which is the right trade for a strip that refreshes ~43 times a
second forever.

## Differences from the esp32c3_neopixel original

| | Original | Here |
|---|---|---|
| Backend | RMT only | `Backend::RMT` (default) or `Backend::SPI` |
| `mem_block_symbols` | 48 (C3 channel size) | 48 on C3, 64 on ESP32 |
| `show()` | `void` | returns `esp_err_t` — a wedged strip fails every frame and the caller wants to log the first one, not abort |
| Copying | implicit | deleted; the handle is owned |

The upstream copy is a separate repository and is not modified by this project.
