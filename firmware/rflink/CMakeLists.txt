# Transitional target: the proven dual-role RF firmware imported from the
# RF-Comms project (Pico2_V2_RF). The ROLE pin selects at boot: high = base
# dongle (USB text protocol <-> nRF24), low = vehicle side (TacticalCore FSM).
#
# This firmware predates the command architecture: its vehicle role still lets
# the Base command motion directly. It is kept building as the working RF
# reference and will be dissolved into airframe/ (radio + relay module) and a
# dongle/ target as the architecture is implemented.

add_executable(wanderer_rflink
    main.cpp
)

set_source_files_properties(
    main.cpp
    PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Wpedantic;-Wno-comment"
)

target_link_libraries(wanderer_rflink
    pico_stdlib
    hardware_gpio
    hardware_spi
    RF24
    wanderer_common
)

pico_enable_stdio_usb(wanderer_rflink 1)
pico_enable_stdio_uart(wanderer_rflink 0)

pico_add_extra_outputs(wanderer_rflink)
