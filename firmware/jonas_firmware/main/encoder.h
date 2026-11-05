#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

// Encoder en cuadratura x4 usando el contador de pulsos por hardware (PCNT).
esp_err_t encoder_init(int idx, gpio_num_t a, gpio_num_t b, bool invert);

// Cuenta acumulada (32 bits, no se desborda en la práctica).
int32_t encoder_get_count(int idx);
