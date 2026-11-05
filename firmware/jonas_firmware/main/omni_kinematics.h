#pragma once

// Cinemática de base omnidireccional de 3 ruedas a 120°.
//
// Marco del robot (REP-103): +x adelante, +y izquierda, +z arriba (wz>0 = giro antihorario).
// Rueda i montada en el ángulo theta_i (desde +x, antihorario), a distancia L del centro.
// Velocidad positiva de rueda = empuja en sentido antihorario visto desde arriba.
//
//   v_i = -sin(theta_i)*vx + cos(theta_i)*vy + L*wz        (v_i = r * w_i)
//
// Montaje de Jonas (triangular): rueda 1 = 0° (al frente, define +x), rueda 2 = 120° (atrás izquierda),
// rueda 3 = -120° = 240° (atrás derecha). Definido en OMNI_THETA_DEG (omni_kinematics.c).
// Vista desde afuera de la rueda (mirando al centro), velocidad positiva = giro HORARIO.

#define OMNI_NUM_WHEELS 3

typedef struct {
    float wheel_radius;   // r [m]
    float base_radius;    // L [m]
    float theta[OMNI_NUM_WHEELS]; // [rad]
} omni_params_t;

void omni_init(omni_params_t *p, float wheel_radius_m, float base_radius_m);

// (vx [m/s], vy [m/s], wz [rad/s]) -> velocidades angulares de rueda [rad/s]
void omni_inverse(const omni_params_t *p, float vx, float vy, float wz, float w_out[OMNI_NUM_WHEELS]);

// velocidades angulares de rueda [rad/s] -> (vx, vy, wz). Para odometría.
void omni_forward(const omni_params_t *p, const float w[OMNI_NUM_WHEELS], float *vx, float *vy, float *wz);
