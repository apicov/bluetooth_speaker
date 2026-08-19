idf_component_register(SRCS "sync_proto.c" "wifi_log.c" "audio_shift.c" "df_servo.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES log esp_timer lwip)
