idf_component_register(SRCS "main.c" "ToF.c" "audio_espnow.c" "ws2812.c"
                       PRIV_REQUIRES nvs_flash esp_event esp_netif esp_wifi led_strip
                       INCLUDE_DIRS "."
                       )