idf_component_register(SRCS "main.c" "sbc_in.c" "beat_detect.c" "visualiser.c" "streamer.c"
                       INCLUDE_DIRS "."
                       REQUIRES dancefloor_sync sbc_decoder esp_driver_uart esp_wifi esp_netif nvs_flash esp_app_format
                                esp_driver_i2s esp_driver_gpio esp_timer)
