cmake_minimum_required(VERSION 3.16)
project(hex_motor_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(hex_motor_example
    src/main.cpp
    src/canopen.cpp
    src/hex_motor.cpp
)

target_include_directories(hex_motor_example PRIVATE src)

# CANopenNode submodule is available for reference or extension.
# To integrate its full stack, implement CO_driver_target.h / CO_driver.c for
# SocketCAN (see https://github.com/CANopenNode/CANopenLinux), then:
#   target_include_directories(hex_motor_example PRIVATE canopennode)
#   target_sources(hex_motor_example PRIVATE canopennode/301/... canopennode/CANopen.c ...)
