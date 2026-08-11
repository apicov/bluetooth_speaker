idf_component_register(SRCS "main.c" "sbc_spi.c" "avrcp_meta.c" "status_led.c"
                    PRIV_REQUIRES bt bt_app_core_utils bredr_app_common_utils a2dp_sink_common_utils
                    PRIV_REQUIRES a2dp_sink_ext_codec_utils
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES dancefloor_sync esp_driver_spi esp_driver_gpio esp_ringbuf)
