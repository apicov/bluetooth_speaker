idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2s esp_driver_pcnt esp_driver_gpio esp_timer)
