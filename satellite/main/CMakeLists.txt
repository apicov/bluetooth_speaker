idf_component_register(SRCS "main.c"
                            "sat_state.c"
                            "net.c"
                            "clock.c"
                            "rx.c"
                            "play.c"
                            "out.c"
                            "servo.c"
                            "telemetry.c"
                       INCLUDE_DIRS "."
                       REQUIRES dancefloor_sync dancefloor_leds sbc_decoder esp_wifi esp_netif nvs_flash esp_app_format
                                esp_driver_i2s esp_driver_gpio esp_timer)
