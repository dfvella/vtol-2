cmake_minimum_required(VERSION 3.13)

include(/usr/share/pico-sdk/external/pico_sdk_import.cmake)

project(flight_controller C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

pico_sdk_init()

include_directories(lib)
add_subdirectory(lib)

add_subdirectory(src)
add_subdirectory(tests)
