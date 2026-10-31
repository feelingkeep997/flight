# Drivers layer - device drivers: IMU, RC receiver, battery
idf_component_register(
    SRCS
        "mpu9250.cpp"
        "imu.cpp"
        "rc.cpp"
        "battery.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        Middlewares
        Bsp
        driver
        esp_timer
        esp_adc
        freertos
        log
)
