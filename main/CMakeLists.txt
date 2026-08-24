idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "." 
    PRIV_REQUIRES nvs_flash esp_event esp_wifi spi_flash esp_driver_gpio esp_driver_i2c led_strip esp_timer
    )