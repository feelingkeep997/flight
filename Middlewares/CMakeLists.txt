# Middlewares layer - hardware-independent math, compat glue and protocols
idf_component_register(
    SRCS
        "cf_common.cpp"
        "time.cpp"
        "util.cpp"
        "mavlink.cpp"
        "cli.cpp"
        "log.cpp"
    # ../Bsp: globals.h pulls board_config.h (pure pin/feature macros, no symbols)
    INCLUDE_DIRS "." "../Bsp"
    REQUIRES
        driver
        esp_timer
        esp_system
        nvs_flash
        freertos
        log
        soc
)
