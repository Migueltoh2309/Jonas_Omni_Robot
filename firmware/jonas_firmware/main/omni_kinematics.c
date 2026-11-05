#include "omni_kinematics.h"

#include <math.h>

static const float OMNI_THETA_DEG[OMNI_NUM_WHEELS] = {0.0f, 120.0f, 240.0f}; // rueda 1 al frente

void omni_init(omni_params_t *p, float wheel_radius_m, float base_radius_m)
{
    p->wheel_radius = wheel_radius_m;
    p->base_radius = base_radius_m;
    for (int i = 0; i < OMNI_NUM_WHEELS; i++) {
        p->theta[i] = OMNI_THETA_DEG[i] * (float)M_PI / 180.0f;
    }
}

void omni_inverse(const omni_params_t *p, float vx, float vy, float wz, float w_out[OMNI_NUM_WHEELS])
{
    for (int i = 0; i < OMNI_NUM_WHEELS; i++) {
        float v = -sinf(p->theta[i]) * vx + cosf(p->theta[i]) * vy + p->base_radius * wz;
        w_out[i] = v / p->wheel_radius;
    }
}

// Inversa cerrada válida para 3 ruedas espaciadas 120° (sum sin^2 = sum cos^2 = 3/2,
// sum sin*cos = sum sin = sum cos = 0).
void omni_forward(const omni_params_t *p, const float w[OMNI_NUM_WHEELS], float *vx, float *vy, float *wz)
{
    float sx = 0.0f, sy = 0.0f, sw = 0.0f;
    for (int i = 0; i < OMNI_NUM_WHEELS; i++) {
        float v = w[i] * p->wheel_radius;
        sx += -sinf(p->theta[i]) * v;
        sy += cosf(p->theta[i]) * v;
        sw += v;
    }
    *vx = sx * (2.0f / 3.0f);
    *vy = sy * (2.0f / 3.0f);
    *wz = sw / (3.0f * p->base_radius);
}
