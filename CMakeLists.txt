# hhh-flight ESP-IDF Project - Keil-style layered layout
#   User -> application -> Drivers -> Bsp -> Middlewares
# Build system stays ESP-IDF/CMake; the layers above are registered as components.

cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    Middlewares
    Bsp
    Drivers
    application
    User
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(hhh-flight)
