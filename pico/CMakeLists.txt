cmake_minimum_required(VERSION 3.13)

# Target the Pico 2 (RP2350). Requires pico-sdk >= 2.0.0.
set(PICO_BOARD pico2 CACHE STRING "Target board")

# Pull in the Pico SDK (must be before project()).
include(pico_sdk_import.cmake)

project(wanderer_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(wanderer_pico
    src/main.c
    src/i2c_peripheral.c
)

target_include_directories(wanderer_pico PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/../protocol   # shared i2c_registers.h
)

target_link_libraries(wanderer_pico
    pico_stdlib
    hardware_i2c
    pico_i2c_slave
)

# Debug console on UART0 (GP0/GP1), not USB.
pico_enable_stdio_uart(wanderer_pico 1)
pico_enable_stdio_usb(wanderer_pico 0)

# Produce .uf2, .elf, map, etc.
pico_add_extra_outputs(wanderer_pico)
