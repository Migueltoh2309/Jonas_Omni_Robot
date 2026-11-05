#pragma once

// Configuración de hardware de Jonas: pines, encoders, límites y PID por defecto.
// Driver: Cytron MDD10A (signo-magnitud: PWM = magnitud, DIR = sentido).
// Motor: Pololu 12 V con encoder, 6533 cuentas por vuelta de la RUEDA (cuadratura x4).

#include <stdbool.h>
#include "driver/gpio.h"

// Motores realmente conectados (los primeros N de la tabla). Subir a 3 al montar el resto.
#define JONAS_NUM_MOTORS 3

typedef struct {
    gpio_num_t pwm;
    gpio_num_t dir;
    gpio_num_t enc_a;
    gpio_num_t enc_b;
    bool invert_motor;    // true si el motor gira al revés de lo esperado
    bool invert_encoder;  // true si el encoder cuenta al revés que el motor
} jonas_wheel_hw_t;

static const jonas_wheel_hw_t JONAS_WHEEL_HW[3] = {
    //  PWM          DIR          ENC A         ENC B         inv_mot inv_enc
    {GPIO_NUM_4,  GPIO_NUM_5,  GPIO_NUM_2,  GPIO_NUM_42, false, false}, // rueda 1 (MDD10A #1 canal 1)
    {GPIO_NUM_6,  GPIO_NUM_7,  GPIO_NUM_41, GPIO_NUM_40, true,  false}, // rueda 2 (MDD10A #1 canal 2)
    {GPIO_NUM_15, GPIO_NUM_16, GPIO_NUM_39, GPIO_NUM_38, true,  true }, // rueda 3 (MDD10A #2 canal 1)
};
// Verificado 2026-11-04 (ruedas en el aire): con velocidad positiva las 3 giran en sentido HORARIO
// vistas desde afuera del robot. invert_motor e invert_encoder son independientes: si se invierte
// el motor hay que invertir también el encoder (o el PI queda con realimentación positiva).

#define JONAS_ENCODER_CPR     6533.0f   // cuentas por vuelta de la rueda
#define JONAS_PWM_FREQ_HZ     20000     // MDD10A soporta hasta 20 kHz
#define JONAS_WHEEL_MAX_RADPS 10.0f     // ~95 RPM; 100 RPM sin carga a 12 V
// Media móvil de la velocidad medida (como en swarmbot/Extra/03_control_PI_velocidad.ino).
// Retardo = (N-1)/2 muestras: 7 -> 30 ms a 100 Hz. Más ventana = más suave pero menos margen de fase.
#define JONAS_VEL_FILTER_WINDOW 7

// Control PI de velocidad por rueda (salida = duty -1..1), ajustable en vivo con /jonas/pid_gains.
// Estructura de swarmbot/Extra/03_control_PI_velocidad.ino (PI + anti-windup condicional, 100 Hz).
//
// Modelo identificado (2026-11-04, motor Pololu 6533 CPR, MDD10A, 12 V, RUEDA EN EL AIRE):
//   G(s) = K / (tau*s + 1),  K ≈ 11.8 rad/s por unidad de duty, tau ≈ 0.08 s, zona muerta ≈ 0.04
// Sintonía por cancelación de polo: Ti = tau, lazo cerrado Tc = 0.1 s -> Kp = tau/(K*Tc), Ki = Kp/tau.
// Resultado medido: t90 ≈ 0.22-0.28 s, sobrepico ≤ 1 %, error en régimen ≈ 0.
// (Ganancias de la referencia SwarmBot convertidas: Kp 0.045 / Ki 0.187 -> t90 ≈ 1.3 s, muy lento.)
// RE-VERIFICAR con el robot en el piso: la inercia y la fricción cambian K y tau.
#define JONAS_PID_KP  0.068f
#define JONAS_PID_KI  0.85f
#define JONAS_PID_KD  0.0f
#define JONAS_PID_KFF 0.0f                         // feedforward desactivado (PI puro)
