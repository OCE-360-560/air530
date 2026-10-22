# air530: UART driver for the Air530 GPS receiver (see README.md).
# In your project's CMakeLists.txt:
#   add_subdirectory(lib/air530)
#   target_link_libraries(<your project> ... hardware_uart air530)
add_library(air530 INTERFACE)

target_sources(air530 INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/air530.c)

target_include_directories(air530 INTERFACE
        ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(air530 INTERFACE
        pico_stdlib hardware_uart hardware_irq)
