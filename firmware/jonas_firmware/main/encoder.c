#include "encoder.h"

#include <stdbool.h>

#include "driver/pulse_cnt.h"

#define ENC_MAX 3
#define PCNT_LIMIT 30000

static pcnt_unit_handle_t s_unit[ENC_MAX];
static bool s_invert[ENC_MAX];

esp_err_t encoder_init(int idx, gpio_num_t a, gpio_num_t b, bool invert)
{
    if (idx < 0 || idx >= ENC_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    const pcnt_unit_config_t ucfg = {
        .high_limit = PCNT_LIMIT,
        .low_limit = -PCNT_LIMIT,
        .flags.accum_count = 1, // acumula al llegar a los límites -> cuenta de 32 bits
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&ucfg, &s_unit[idx]));

    const pcnt_glitch_filter_config_t filt = {.max_glitch_ns = 1000};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_unit[idx], &filt));

    // Cuadratura x4: cada canal cuenta ambos flancos de su señal según el nivel de la otra.
    pcnt_channel_handle_t ch_a, ch_b;
    const pcnt_chan_config_t ca = {.edge_gpio_num = a, .level_gpio_num = b};
    const pcnt_chan_config_t cb = {.edge_gpio_num = b, .level_gpio_num = a};
    ESP_ERROR_CHECK(pcnt_new_channel(s_unit[idx], &ca, &ch_a));
    ESP_ERROR_CHECK(pcnt_new_channel(s_unit[idx], &cb, &ch_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Pull-ups débiles: evitan cuentas fantasma si el encoder está desconectado.
    gpio_pullup_en(a);
    gpio_pullup_en(b);

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_unit[idx], PCNT_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_unit[idx], -PCNT_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(s_unit[idx]));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_unit[idx]));
    ESP_ERROR_CHECK(pcnt_unit_start(s_unit[idx]));

    s_invert[idx] = invert;
    return ESP_OK;
}

int32_t encoder_get_count(int idx)
{
    if (idx < 0 || idx >= ENC_MAX || s_unit[idx] == NULL) {
        return 0;
    }
    int c = 0;
    pcnt_unit_get_count(s_unit[idx], &c);
    return s_invert[idx] ? -c : c;
}
