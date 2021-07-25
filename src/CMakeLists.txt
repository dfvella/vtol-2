########### Add Libraries ##########
add_library(fir_filter fir_filter.h fir_filter.c)
add_library(flight_controller flight_controller.h flight_controller.c)
add_library(logging logging.h logging.c)
add_library(pid_controller pid_controller.h pid_controller.c)
add_library(pwm pwm.h pwm.c)
add_library(reboot reboot.h reboot.c)

########## Add Exetuables ##########
add_executable(main main.c constants.h)

########## Pico Config pwm ##########
pico_generate_pio_header(pwm ${CMAKE_CURRENT_LIST_DIR}/pwm.S)

########## Pico Config main ##########
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
pico_add_extra_outputs(main)

########## Link Libraries ##########
target_link_libraries(fir_filter
    pico_stdlib
)
target_link_libraries(flight_controller
    pico_stdlib
    3dmath
    pid_controller
)
target_link_libraries(logging
    pico_stdlib
    hardware_flash
    hardware_sync
    flight_controller
)
target_link_libraries(pid_controller
    pico_stdlib
    fir_filter
)
target_link_libraries(pwm
    pico_stdlib
    hardware_pio
)
target_link_libraries(reboot
    pico_stdlib
)
target_link_libraries(main
    pico_stdlib
    hardware_gpio
    hardware_i2c
    mpu6050
    ar610
    logging
    pwm
    reboot
)
