# add executable test_radio
add_executable(imu 
    imu.c
)

pico_enable_stdio_usb(imu 1)
pico_enable_stdio_uart(imu 0)

pico_add_extra_outputs(imu)

target_link_libraries(imu
    pico_stdlib
    hardware_i2c
    mpu6050
)

# add exectable test_radio
add_executable(radio
    radio.c
)

pico_enable_stdio_usb(radio 1)
pico_enable_stdio_uart(radio 0)

pico_add_extra_outputs(radio)

target_link_libraries(radio
    pico_stdlib
    ar610
)
