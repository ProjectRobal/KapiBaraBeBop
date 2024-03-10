idf_component_register(SRCS "WiFiManager.c" "HttpService.c"
                    INCLUDE_DIRS "include" REQUIRES esp_event nvs_flash esp_wifi esp_http_server freertos PRIV_REQUIRES esp_littlefs)
