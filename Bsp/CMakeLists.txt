# Bsp layer - board support: pin config, WiFi, web console
idf_component_register(
    SRCS
        "wifi.cpp"
        "web_rc.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        Middlewares
        esp_wifi
        esp_event
        esp_netif
        lwip
        nvs_flash
        esp_http_server
        driver
        esp_timer
        esp_system
        freertos
        log
)
