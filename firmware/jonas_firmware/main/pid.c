#include "pid.h"

void pid_reset(pid_state_t *s)
{
    s->integ = 0.0f;
    s->prev_meas = 0.0f;
    s->init = false;
}

float pid_step(pid_state_t *s, const pid_gains_t *g, float target, float meas, float dt)
{
    const float lim = g->out_limit;
    const float err = target - meas;

    float d = 0.0f;
    if (s->init && dt > 0.0f) {
        d = -(meas - s->prev_meas) / dt;
    }
    s->prev_meas = meas;
    s->init = true;

    const float u_base = g->kff * target + g->kp * err + g->kd * d;

    float integ_new = s->integ + g->ki * err * dt;
    if (integ_new > lim) integ_new = lim;
    if (integ_new < -lim) integ_new = -lim;

    float u = u_base + integ_new;
    // Anti-windup: no integrar si eso empuja más allá de la saturación.
    if ((u > lim && err > 0.0f) || (u < -lim && err < 0.0f)) {
        u = u_base + s->integ;
    } else {
        s->integ = integ_new;
    }

    if (u > lim) u = lim;
    if (u < -lim) u = -lim;
    return u;
}
