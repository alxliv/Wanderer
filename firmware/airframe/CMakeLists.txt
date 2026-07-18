# Wanderer airframe — the tactical layer on the rover's Pico 2.
#
# The previous I2C-cockpit firmware is parked in legacy/ (the architecture
# moved the cockpit to UART — see docs/Wanderer_Command_Architecture.md §8).
# Targets: wanderer_airframe (the flight firmware -- UART cockpit, tactical
# FSM, motors, encoders) and the one-shot motor bring-up test.

# The airframe flight firmware. Cockpit protocol on UART0 (GP0/GP1, raw
# lines, not stdio); bench logs on USB CDC.
add_executable(wanderer_airframe
    src/main.cpp
    src/encoders.c
    src/encoder_math.c
    src/motor_output.c
    src/motors.c
)

pico_generate_pio_header(wanderer_airframe
    ${CMAKE_CURRENT_LIST_DIR}/src/quadrature_encoder.pio
)

target_include_directories(wanderer_airframe PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(wanderer_airframe
    wanderer_common
    pico_stdlib
    hardware_clocks
    hardware_pio
    hardware_pwm
    hardware_uart
)

pico_enable_stdio_uart(wanderer_airframe 0)
pico_enable_stdio_usb(wanderer_airframe 1)
pico_add_extra_outputs(wanderer_airframe)

# Standalone, one-shot physical motor bring-up test. Flash only while the robot
# is raised so its wheels can turn freely.
add_executable(wanderer_motor_test
    tests/motor_hardware_test.c
    src/encoders.c
    src/motor_output.c
    src/motors.c
)

pico_generate_pio_header(wanderer_motor_test
    ${CMAKE_CURRENT_LIST_DIR}/src/quadrature_encoder.pio
)

target_include_directories(wanderer_motor_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(wanderer_motor_test
    pico_stdlib
    hardware_clocks
    hardware_pio
    hardware_pwm
)

# The hardware test is manually armed through the Pico's USB CDC serial port.
# Keep UART0 free so no external USB-to-UART adapter is required.
pico_enable_stdio_uart(wanderer_motor_test 0)
pico_enable_stdio_usb(wanderer_motor_test 1)
pico_add_extra_outputs(wanderer_motor_test)
