cmake_minimum_required(VERSION 3.28)
set(PICO_BOARD pico_w)

include(pico_sdk_import.cmake)
project(pico_interface_fmsynth C CXX ASM)


set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(Perl)
if(NOT PERL_FOUND)
    message(FATAL_ERROR "Perl is needed for generating the fsdata.c file")
endif()




set(MAKE_FS_DATA_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/external/makefsdata)

if (NOT EXISTS ${MAKE_FS_DATA_SCRIPT})
        file(DOWNLOAD
                https://raw.githubusercontent.com/lwip-tcpip/lwip/e799c266facc3c70190676eccad49d6c2db2caac/src/apps/http/makefsdata/makefsdata
                ${MAKE_FS_DATA_SCRIPT}
                )
endif()
message("Running makefsdata script")
execute_process(COMMAND
        perl ${MAKE_FS_DATA_SCRIPT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        ECHO_OUTPUT_VARIABLE
        ECHO_ERROR_VARIABLE
        )
file(RENAME fsdata.c my_fsdata.c)


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")


pico_sdk_init()

add_executable(pico_interface_fmsynth
        main.cpp
        cgi.cpp
        ssi.cpp
        MEMLSerial_Pico.cpp
        )


# create map/bin/hex file etc.
pico_enable_stdio_usb(pico_interface_fmsynth 1)
pico_enable_stdio_uart(pico_interface_fmsynth 0)
pico_add_extra_outputs(pico_interface_fmsynth)
pico_add_extra_outputs(pico_interface_fmsynth)
target_include_directories(pico_interface_fmsynth PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(pico_interface_fmsynth pico_stdlib hardware_adc pico_multicore 
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_http)

# add url via pico_set_program_url
# example_auto_set_url(pico_interface_fmsynth)
