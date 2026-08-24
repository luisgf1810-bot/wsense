idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "include" 
    PRIV_REQUIRES nvs_flash esp_event esp_wifi spi_flash esp_driver_gpio led_strip
    )