# application layer (formerly App) - flight application logic (control/estimation/mixer/params)
idf_component_register(
    SRCS
        "control.cpp"
        "estimate.cpp"
        "motors.cpp"
        "safety.cpp"
        "led.cpp"
        "parameters.cpp"
        "freertos_demo.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        Middlewares
        Bsp
        Drivers
        driver
        esp_timer
        nvs_flash
        freertos
        esp_system
        log
)
