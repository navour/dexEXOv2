idf_component_register(SRCS "power_manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_timer esp_adc nvs_flash
                    PRIV_REQUIRES driver)
