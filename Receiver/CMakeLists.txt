add_executable(multicore_dds nrf24_driver.c pin_manager.c spi_manager.c multicore_dds.c)

# Add pico_multicore which is required for multicore functionality
target_link_libraries(multicore_dds pico_stdlib pico_bootsel_via_double_reset pico_multicore hardware_sync hardware_spi pico_bootsel_via_double_reset hardware_pio 
			hardware_dma hardware_adc hardware_irq )

pico_enable_stdio_usb(multicore_dds 1)
pico_enable_stdio_uart(multicore_dds 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(multicore_dds)