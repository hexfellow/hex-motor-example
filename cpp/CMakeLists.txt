cmake_minimum_required(VERSION 3.16)

# Cross-compilation toolchain (Buildroot aarch64) configure starts
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(TOOLCHAIN_PREFIX "$ENV{HOME}/buildroot/output/host/bin/aarch64-linux-")
set(CMAKE_SYSROOT    "$ENV{HOME}/buildroot/output/host/aarch64-buildroot-linux-gnu/sysroot")

set(CMAKE_C_COMPILER   "${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PREFIX}g++")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Cross-compilation toolchain configure ends

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
