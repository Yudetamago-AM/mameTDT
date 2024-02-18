add_executable(main3 main3.c)

pico_generate_pio_header(main3 ${CMAKE_CURRENT_LIST_DIR}/pio_out.pio)

target_link_libraries(main3 pico_stdlib hardware_spi hardware_pio)

pico_enable_stdio_usb(main3 1)
pico_enable_stdio_uart(main3 0)

pico_add_extra_outputs(main3)
