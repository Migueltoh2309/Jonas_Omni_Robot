#include "motor.h"

#include <math.h>
#include <stdbool.h>

#include "driver/ledc.h"

#define MOTOR_MAX 3
#define PWM_RES LEDC_TIMER_10_BIT
#define PWM_MAX_DUTY ((1 << 10) - 1)

static gpio_num_t s_dir[MOTOR_MAX];
static bool s_invert[MOTOR_MAX];
static bool s_timer_ready;

void motor_force_low(gpio_num_t pwm, gpio_num_t dir)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << pwm) | (1ULL << dir),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cfg);
    gpio_set_level(pwm, 0);
    gpio_set_level(dir, 0);
}

esp_err_t motor_init(int idx, gpio_num_t pwm, gpio_num_t dir, int freq_hz, bool invert)
{
    if (idx < 0 || idx >= MOTOR_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_timer_ready) {
        const ledc_timer_config_t t = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = PWM_RES,
            .timer_num = LEDC_TIMER_0,
            .freq_hz = freq_hz,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ESP_ERROR_CHECK(ledc_timer_config(&t));
        s_timer_ready = true;
    }
    const ledc_channel_config_t c = {
        .gpio_num = pwm,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = (ledc_channel_t)idx,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&c));

    s_dir[idx] = dir;
    s_invert[idx] = invert;
    gpio_set_direction(dir, GPIO_MODE_OUTPUT);
    gpio_set_level(dir, 0);
    return ESP_OK;
}

void motor_set(int idx, float duty)
{
    if (idx < 0 || idx >= MOTOR_MAX) {
        return;
    }
    if (duty > 1.0f) duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;
    if (s_invert[idx]) duty = -duty;

    gpio_set_level(s_dir[idx], duty >= 0.0f ? 1 : 0);
    uint32_t d = (uint32_t)lroundf(fabsf(duty) * PWM_MAX_DUTY);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx, d);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx);
}
