add_executable(main
    constants.h
    main.c
    timing.h
    timing.c
    logging.h
    logging.c
    flight_controller.h
    flight_controller.c
    pwm.h
    pwm.c
    pid_controller.h
    pid_controller.c
    fir_filter.h
    fir_filter.c
)

pico_generate_pio_header(main ${CMAKE_CURRENT_LIST_DIR}/pwm.S)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("Enabling stdio over USB serial port")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG")
    pico_enable_stdio_usb(main 1)
else()
    pico_enable_stdio_usb(main 0)
endif()

pico_enable_stdio_uart(main 0)

pico_add_extra_outputs(main)

target_link_libraries(main
    pico_stdlib
    hardware_i2c
    hardware_pio
    hardware_flash
    hardware_sync
    mpu6050
    ar610
)
