idf_component_register(SRCS  "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES icm20948 lsm9ds1 mpu9250 vqf mag_calibration wifi_transport power_manager ota_update led_manager
                    PRIV_REQUIRES esp_timer nvs_flash driver)
