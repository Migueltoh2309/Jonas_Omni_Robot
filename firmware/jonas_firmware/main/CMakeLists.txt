idf_component_register(
    SRCS "main.c" "uart_transport.c" "omni_kinematics.c" "motor.c" "encoder.c" "pid.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_uart esp_driver_ledc esp_driver_pcnt esp_driver_gpio esp_timer
)
