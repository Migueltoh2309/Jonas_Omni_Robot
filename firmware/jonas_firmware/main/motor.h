#pragma once

#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

// Pone PWM y DIR en bajo como salida digital. Llamar lo antes posible al arrancar.
void motor_force_low(gpio_num_t pwm, gpio_num_t dir);

esp_err_t motor_init(int idx, gpio_num_t pwm, gpio_num_t dir, int freq_hz, bool invert);

// duty en [-1, 1]; el signo define el sentido (pin DIR), la magnitud el PWM.
void motor_set(int idx, float duty);
