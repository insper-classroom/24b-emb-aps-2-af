add_executable(main
        hc05.c
        main.c
)

target_link_libraries(main pico_stdlib freertos hardware_adc hardware_spi hardware_uart)
pico_add_extra_outputs(main)
