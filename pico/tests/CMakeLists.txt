cmake_minimum_required(VERSION 3.13)
project(wanderer_pico_unit_tests C)

set(CMAKE_C_STANDARD 11)
enable_testing()

add_executable(test_motor_output
    test_motor_output.c
    ../src/motor_output.c
)

target_include_directories(test_motor_output PRIVATE
    ../src
)

add_test(NAME motor_output COMMAND test_motor_output)
