idf_component_register(
    SRCS 
        "motion_controller.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        stepper_driver sensor_drivers esp_timer esp_ringbuf
)