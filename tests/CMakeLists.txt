include_directories(${CMAKE_CURRENT_LIST_DIR}/../src)

########## Add Executable test_radio ##########
add_executable(imu imu.c)
pico_enable_stdio_usb(imu 1)
pico_enable_stdio_uart(imu 0)
pico_add_extra_outputs(imu)
target_link_libraries(imu
    pico_stdlib
    hardware_i2c
    mpu6050
)

########## Add Exectable test_radio ##########
add_executable(radio radio.c)
pico_enable_stdio_usb(radio 1)
pico_enable_stdio_uart(radio 0)
pico_add_extra_outputs(radio)
target_link_libraries(radio
    pico_stdlib
    ar610
)

########## Add Flight Controller Regression Test ##########
add_executable(sim sim.c sim_input.h ../src/constants.h)
pico_enable_stdio_usb(sim 1)
pico_enable_stdio_uart(sim 0)
pico_add_extra_outputs(sim)
target_link_libraries(sim
    pico_stdlib
    flight_controller
    logging
    reboot
)
