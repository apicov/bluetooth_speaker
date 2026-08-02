idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES dancefloor_sync esp_wifi esp_netif nvs_flash esp_driver_gpio esp_timer)
