cmake_minimum_required(VERSION 3.28)

include(pico_sdk_import.cmake)
project(pico_interface_fmsynth C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_VERBOSE_MAKEFILE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")


pico_sdk_init()

add_executable(pico_interface_fmsynth
        fmsynth-interface.cpp
        )


# create map/bin/hex file etc.
pico_enable_stdio_usb(pico_interface_fmsynth 1)
pico_enable_stdio_uart(pico_interface_fmsynth 0)
pico_add_extra_outputs(pico_interface_fmsynth)
pico_add_extra_outputs(pico_interface_fmsynth)

target_link_libraries(pico_interface_fmsynth pico_stdlib hardware_adc pico_multicore)

# add url via pico_set_program_url
# example_auto_set_url(pico_interface_fmsynth)
