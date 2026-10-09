idf_component_register(SRCS "ota_update.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_http_server app_update esp_app_format log
                    PRIV_REQUIRES wifi_transport)
