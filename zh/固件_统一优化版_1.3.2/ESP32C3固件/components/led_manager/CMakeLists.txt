# LED Manager Component
idf_component_register(SRCS "led_manager.c"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos esp_timer ws2812_driver)