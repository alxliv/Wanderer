cmake_minimum_required(VERSION 3.13)
project(wanderer_common_unit_tests CXX)

# Host-side unit tests for firmware/common -- builds WITHOUT the Pico SDK,
# on Linux/Windows/macOS. Same pattern as firmware/airframe/tests.
#
#   cmake -S firmware/common/tests -B build_common_tests
#   cmake --build build_common_tests
#   ctest --test-dir build_common_tests
#   (Visual Studio generator: add -C Debug to the ctest line)
# or run cmake with Ninja generator:
# cmake -S firmware/common/tests -B build_common_tests -G Ninja

set(CMAKE_CXX_STANDARD 17)
enable_testing()

add_executable(test_tactical
    ../test_tactical.cpp
    ../tactical.cpp
)
target_include_directories(test_tactical PRIVATE ..)
add_test(NAME tactical COMMAND test_tactical)

add_executable(test_codec
    test_codec.cpp
    ../cockpit_codec.cpp
)
target_include_directories(test_codec PRIVATE ..)
target_compile_definitions(test_codec PRIVATE
    VECTORS_PATH="${CMAKE_CURRENT_LIST_DIR}/../../../protocol/cockpit_vectors.txt")
add_test(NAME codec COMMAND test_codec)

add_executable(test_handler
    test_handler.cpp
    ../cockpit_handler.cpp
    ../cockpit_codec.cpp
    ../tactical.cpp
)
target_include_directories(test_handler PRIVATE ..)
add_test(NAME handler COMMAND test_handler)
