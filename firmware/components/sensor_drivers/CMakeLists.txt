idf_component_register(
    SRCS 
        "as5600.c"
        "ina226.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        esp_driver_i2c esp_driver_gpio
)