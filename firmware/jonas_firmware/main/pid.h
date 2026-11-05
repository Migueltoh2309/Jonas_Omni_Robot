#pragma once

#include <stdbool.h>

typedef struct {
    float kp, ki, kd, kff;   // kff: feedforward (salida por unidad de consigna)
    float out_limit;         // salida en [-out_limit, out_limit]
} pid_gains_t;

typedef struct {
    float integ;      // término integral ya multiplicado por ki
    float prev_meas;
    bool init;
} pid_state_t;

void pid_reset(pid_state_t *s);

// Derivada sobre la medición (sin "patada" al cambiar la consigna) y anti-windup condicional.
float pid_step(pid_state_t *s, const pid_gains_t *g, float target, float meas, float dt);
