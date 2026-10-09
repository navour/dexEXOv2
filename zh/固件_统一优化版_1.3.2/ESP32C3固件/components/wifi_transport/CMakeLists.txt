idf_component_register(SRCS "wifi_transport.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_wifi esp_netif esp_event log nvs_flash
                    PRIV_REQUIRES lwip)

idf_build_get_property(project_ver PROJECT_VER)
target_compile_definitions(__idf_wifi_transport PRIVATE "APP_VERSION=\"${project_ver}\"")
