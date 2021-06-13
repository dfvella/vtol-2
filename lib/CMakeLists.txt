add_library(mpu6050
    mpu6050.h
    mpu6050.c
)

target_link_libraries(mpu6050
    pico_stdlib
    hardware_i2c
)

add_library(ar610
    ar610.h
    ar610.c
)

target_link_libraries(ar610
    pico_stdlib
    hardware_pwm
)
