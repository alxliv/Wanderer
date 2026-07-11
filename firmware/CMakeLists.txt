cmake_minimum_required(VERSION 3.13)

# Target the Pico 2 (RP2350). Requires pico-sdk >= 2.0.0.
set(PICO_BOARD pico2 CACHE STRING "Target board")

# Pull in the Pico SDK (must be before project()).
include(pico_sdk_import.cmake)

project(wanderer_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# nRF24 radio driver (git submodule, pinned).
include(${CMAKE_CURRENT_LIST_DIR}/lib/RF24/CMakeLists.txt)

# Shared across all firmware targets: RF binary protocol frames and the
# TacticalCore FSM. Header-only for now.
add_library(wanderer_common INTERFACE)
target_include_directories(wanderer_common INTERFACE ${CMAKE_CURRENT_LIST_DIR}/common)

add_subdirectory(airframe)
add_subdirectory(rflink)
