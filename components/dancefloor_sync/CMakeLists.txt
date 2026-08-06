idf_component_register(SRCS "sync_proto.c" "wifi_log.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES log esp_timer lwip)
