# Example and first hardware check for the air530 driver.
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(air530_example C CXX ASM)

pico_sdk_init()

# The driver is one folder up here; in your project it is lib/air530.
add_subdirectory(.. air530)

add_executable(air530_example air530_example.c)

target_link_libraries(air530_example pico_stdlib hardware_uart air530)

pico_enable_stdio_usb(air530_example 1)
pico_enable_stdio_uart(air530_example 0)

pico_add_extra_outputs(air530_example)
