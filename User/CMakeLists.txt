# User layer - entry point & task scheduling (Keil "User": main.c)
idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        Middlewares
        Bsp
        Drivers
        application
        driver
        esp_timer
        nvs_flash
        freertos
        esp_system
        log
)
