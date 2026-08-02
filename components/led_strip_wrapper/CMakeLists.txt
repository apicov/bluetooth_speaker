idf_component_register(
    SRCS "led_strip_wrapper.cpp"
    INCLUDE_DIRS "."
    REQUIRES "led_strip" "esp_driver_gpio" "esp_driver_spi" "esp_driver_rmt"
)

target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++20)
