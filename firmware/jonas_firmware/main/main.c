// Jonas - firmware base ESP32-S3
//
// micro-ROS (Jazzy) por UART0 hacia el micro_ros_agent de la Raspberry Pi.
// Control de velocidad por rueda a CONFIG_JONAS_CONTROL_HZ: encoder (PCNT) -> PI -> Cytron MDD10A.
// La cinemática (/cmd_vel -> ruedas) y la odometría (ticks -> /odom) se hacen en la Raspi (ROS 2).
//
// Entradas (de mayor a menor prioridad; cada comando vale mientras llegue al menos cada
// CONFIG_JONAS_CMD_TIMEOUT_MS, si no -> motores a 0):
//   /jonas/wheel_pwm        std_msgs/Float32MultiArray  duty por rueda [-1..1], LAZO ABIERTO (pruebas)
//   /jonas/wheel_vel_cmd    std_msgs/Float32MultiArray  velocidad por rueda [rad/s], con PI
// Configuración en vivo (se pierde al reiniciar):
//   /jonas/pid_gains        std_msgs/Float32MultiArray  [kp, ki, kd] o [kp, ki, kd, kff]
//   /jonas/wheel_state_rate std_msgs/Int32              Hz de /jonas/wheel_state (divisor de 50; 0 = apagado)
// Salidas:
//   /jonas/wheel_ticks      std_msgs/Int32MultiArray    50 Hz: [t_ms, ticks1, ticks2, ticks3] (para odometría)
//   /jonas/wheel_state      std_msgs/Float32MultiArray  25 Hz por defecto: [objetivo x3, medida x3, duty x3]
//   /jonas/status           std_msgs/String             al conectar y cada 2 s

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/string.h>

#include "encoder.h"
#include "jonas_hw.h"
#include "motor.h"
#include "pid.h"
#include "uart_transport.h"

static const char *TAG = "jonas";

#define RCCHECK(fn)                                                                  \
    do {                                                                             \
        rcl_ret_t rc_ = (fn);                                                        \
        if (rc_ != RCL_RET_OK) {                                                     \
            ESP_LOGE(TAG, "%s:%d fallo %d", __FILE__, __LINE__, (int)rc_);           \
            return false;                                                            \
        }                                                                            \
    } while (0)
#define RCSOFTCHECK(fn)            \
    do {                           \
        rcl_ret_t rc_ = (fn);      \
        (void)rc_;                 \
    } while (0)

#define NW 3                        // ruedas (posiciones en la tabla JONAS_WHEEL_HW)
#define PUB_PERIOD_MS 20            // timer de publicación: 50 Hz
#define PUB_HZ (1000 / PUB_PERIOD_MS)
#define STATUS_PERIOD_TICKS 100     // /jonas/status cada 100 ciclos = 2 s
#define WHEEL_STATE_DEFAULT_HZ 25

// ---------------------------------------------------------------- estado compartido

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    float vals[NW];
    int64_t stamp_us;   // 0 = nunca recibido
} timed_cmd_t;

static timed_cmd_t s_wheel_vel_cmd;
static timed_cmd_t s_wheel_pwm_cmd;
static pid_gains_t s_gains = {
    .kp = JONAS_PID_KP, .ki = JONAS_PID_KI, .kd = JONAS_PID_KD, .kff = JONAS_PID_KFF, .out_limit = 1.0f,
};
static volatile bool s_agent_connected;

// Salidas del lazo de control
static float s_state[3 * NW];       // /jonas/wheel_state: objetivo, medida, duty
static int32_t s_ticks[NW];         // /jonas/wheel_ticks: cuentas acumuladas de encoder
static int32_t s_ticks_ms;          // instante (ms desde el arranque) en que se leyeron los encoders

static volatile int s_state_div = PUB_HZ / WHEEL_STATE_DEFAULT_HZ;   // publicar cada N ciclos (0 = apagado)

// Diagnóstico para /jonas/status (reemplaza a los logs cuando solo hay un cable: el UART)
typedef enum { MODE_STOP, MODE_PWM, MODE_VEL } ctrl_mode_t;
static const char *const MODE_NAMES[] = {"parado", "pwm", "vel"};
static volatile ctrl_mode_t s_mode = MODE_STOP;
static int s_connections;       // conexiones al Agent desde el arranque
static int s_create_failures;   // intentos fallidos de crear entidades

// ---------------------------------------------------------------- lazo de control

static bool is_fresh(int64_t stamp_us, int64_t now_us)
{
    return stamp_us != 0 && (now_us - stamp_us) < (int64_t)CONFIG_JONAS_CMD_TIMEOUT_MS * 1000;
}

// Si alguna rueda supera el máximo, escala todas por igual (conserva la dirección del movimiento).
static void limit_wheel_speeds(float w[NW])
{
    float max_abs = 0.0f;
    for (int i = 0; i < NW; i++) {
        max_abs = fmaxf(max_abs, fabsf(w[i]));
    }
    if (max_abs > JONAS_WHEEL_MAX_RADPS) {
        float k = JONAS_WHEEL_MAX_RADPS / max_abs;
        for (int i = 0; i < NW; i++) {
            w[i] *= k;
        }
    }
}

static void control_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_JONAS_CONTROL_HZ);
    const float rad_per_count = 2.0f * (float)M_PI / JONAS_ENCODER_CPR;

    pid_state_t pid[NW];
    int32_t count[NW] = {0};
    int32_t last_count[NW] = {0};
    float w_meas[NW] = {0};
    // Media móvil por rueda (buffer circular + suma acumulada)
    float filt_buf[NW][JONAS_VEL_FILTER_WINDOW] = {{0}};
    float filt_sum[NW] = {0};
    int filt_idx = 0;
    for (int i = 0; i < NW; i++) {
        pid_reset(&pid[i]);
        last_count[i] = encoder_get_count(i);
    }

    int64_t last_us = esp_timer_get_time();
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, period);
        const int64_t now = esp_timer_get_time();
        const float dt = (now - last_us) * 1e-6f;
        last_us = now;

        // 1) Encoders: cuenta acumulada y velocidad medida
        for (int i = 0; i < JONAS_NUM_MOTORS; i++) {
            count[i] = encoder_get_count(i);
            int32_t delta = count[i] - last_count[i];
            last_count[i] = count[i];
            float w_raw = (dt > 0.0f) ? delta * rad_per_count / dt : 0.0f;
            filt_sum[i] += w_raw - filt_buf[i][filt_idx];
            filt_buf[i][filt_idx] = w_raw;
            w_meas[i] = filt_sum[i] / JONAS_VEL_FILTER_WINDOW;
        }
        filt_idx = (filt_idx + 1) % JONAS_VEL_FILTER_WINDOW;

        // 2) Comandos vigentes
        timed_cmd_t pwm_cmd, vel_cmd;
        pid_gains_t g;
        taskENTER_CRITICAL(&s_lock);
        pwm_cmd = s_wheel_pwm_cmd;
        vel_cmd = s_wheel_vel_cmd;
        g = s_gains;
        taskEXIT_CRITICAL(&s_lock);

        float target[NW] = {0};
        float duty[NW] = {0};
        const bool ok = s_agent_connected;
        ctrl_mode_t mode = MODE_STOP;

        if (ok && is_fresh(pwm_cmd.stamp_us, now)) {
            mode = MODE_PWM;   // lazo abierto
            for (int i = 0; i < NW; i++) {
                duty[i] = fmaxf(-1.0f, fminf(1.0f, pwm_cmd.vals[i]));
                pid_reset(&pid[i]);
            }
        } else {
            const bool active = ok && is_fresh(vel_cmd.stamp_us, now);
            if (active) {
                mode = MODE_VEL;
                memcpy(target, vel_cmd.vals, sizeof(target));
                limit_wheel_speeds(target);
            }
            for (int i = 0; i < NW; i++) {
                if (!active || target[i] == 0.0f) {
                    duty[i] = 0.0f;
                    pid_reset(&pid[i]);
                } else {
                    duty[i] = pid_step(&pid[i], &g, target[i], w_meas[i], dt);
                }
            }
        }
        s_mode = mode;

        // 3) Actuar
        for (int i = 0; i < JONAS_NUM_MOTORS; i++) {
            motor_set(i, duty[i]);
        }

        // 4) Publicar al resto del sistema
        taskENTER_CRITICAL(&s_lock);
        for (int i = 0; i < NW; i++) {
            const bool present = i < JONAS_NUM_MOTORS;
            s_state[i] = target[i];
            s_state[NW + i] = present ? w_meas[i] : 0.0f;
            s_state[2 * NW + i] = present ? duty[i] : 0.0f;
            s_ticks[i] = present ? count[i] : 0;
        }
        s_ticks_ms = (int32_t)(now / 1000);
        taskEXIT_CRITICAL(&s_lock);
    }
}

// ---------------------------------------------------------------- micro-ROS

// rclc guarda un PUNTERO al allocator (support y executor): debe vivir siempre, no puede ser local.
static rcl_allocator_t s_allocator;
static rclc_support_t s_support;
static rcl_node_t s_node;
static rcl_subscription_t s_sub_wheel_vel, s_sub_wheel_pwm, s_sub_gains, s_sub_state_rate;
static rcl_publisher_t s_pub_state, s_pub_ticks, s_pub_status;
static rcl_timer_t s_pub_timer;
static rclc_executor_t s_executor;

static std_msgs__msg__Float32MultiArray s_vel_msg, s_pwm_msg, s_gains_msg, s_state_msg;
static float s_vel_buf[NW], s_pwm_buf[NW], s_gains_buf[4], s_state_buf[3 * NW];
static std_msgs__msg__Int32MultiArray s_ticks_msg;
static int32_t s_ticks_buf[1 + NW];
static std_msgs__msg__Int32 s_state_rate_msg;
static std_msgs__msg__String s_status_msg;
static char s_status_buf[160];

static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:   return "encendido";
    case ESP_RST_SW:        return "reinicio por software";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "watchdog de interrupciones";
    case ESP_RST_TASK_WDT:  return "watchdog de tareas";
    case ESP_RST_WDT:       return "watchdog";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (bajo voltaje)";
    case ESP_RST_EXT:       return "pin de reset";
    case ESP_RST_DEEPSLEEP: return "deep sleep";
    default:                return "desconocido";
    }
}

static int state_rate_hz(void)
{
    int div = s_state_div;
    return div > 0 ? PUB_HZ / div : 0;
}

static void publish_status(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void publish_status(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(s_status_buf, sizeof(s_status_buf), fmt, ap);
    va_end(ap);
    if (n < 0) {
        return;
    }
    s_status_msg.data.data = s_status_buf;
    s_status_msg.data.size = strlen(s_status_buf);
    s_status_msg.data.capacity = sizeof(s_status_buf);
    RCSOFTCHECK(rcl_publish(&s_pub_status, &s_status_msg, NULL));
}

static void init_array_msg(std_msgs__msg__Float32MultiArray *m, float *buf, size_t cap, size_t size)
{
    memset(m, 0, sizeof(*m));
    m->data.data = buf;
    m->data.capacity = cap;
    m->data.size = size;
}

static void store_wheel_cmd(timed_cmd_t *dst, const std_msgs__msg__Float32MultiArray *m)
{
    timed_cmd_t c = {0};
    for (size_t i = 0; i < NW && i < m->data.size; i++) {
        c.vals[i] = m->data.data[i];
    }
    c.stamp_us = esp_timer_get_time();
    taskENTER_CRITICAL(&s_lock);
    *dst = c;
    taskEXIT_CRITICAL(&s_lock);
}

static void wheel_vel_cb(const void *msgin)
{
    store_wheel_cmd(&s_wheel_vel_cmd, (const std_msgs__msg__Float32MultiArray *)msgin);
}

static void wheel_pwm_cb(const void *msgin)
{
    store_wheel_cmd(&s_wheel_pwm_cmd, (const std_msgs__msg__Float32MultiArray *)msgin);
}

static void gains_cb(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *m = (const std_msgs__msg__Float32MultiArray *)msgin;
    if (m->data.size < 3) {
        return;
    }
    taskENTER_CRITICAL(&s_lock);
    s_gains.kp = m->data.data[0];
    s_gains.ki = m->data.data[1];
    s_gains.kd = m->data.data[2];
    if (m->data.size >= 4) {
        s_gains.kff = m->data.data[3];
    }
    pid_gains_t g = s_gains;
    taskEXIT_CRITICAL(&s_lock);
    publish_status("PID: kp=%.4f ki=%.4f kd=%.4f kff=%.4f", g.kp, g.ki, g.kd, g.kff);
}

// Hz pedidos -> divisor entero de 50 Hz (redondeado). 0 o negativo = apagado.
static void state_rate_cb(const void *msgin)
{
    const int hz = ((const std_msgs__msg__Int32 *)msgin)->data;
    int div = 0;
    if (hz > 0) {
        div = (PUB_HZ + hz / 2) / hz;
        if (div < 1) div = 1;
        if (div > PUB_HZ) div = PUB_HZ;
    }
    s_state_div = div;
    publish_status("wheel_state a %d Hz", state_rate_hz());
}

static void pub_timer_cb(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) {
        return;
    }
    static int tick;
    tick++;

    taskENTER_CRITICAL(&s_lock);
    s_ticks_buf[0] = s_ticks_ms;
    memcpy(&s_ticks_buf[1], s_ticks, sizeof(s_ticks));
    memcpy(s_state_buf, s_state, sizeof(s_state_buf));
    taskEXIT_CRITICAL(&s_lock);

    RCSOFTCHECK(rcl_publish(&s_pub_ticks, &s_ticks_msg, NULL));

    const int div = s_state_div;
    if (div > 0 && tick % div == 0) {
        RCSOFTCHECK(rcl_publish(&s_pub_state, &s_state_msg, NULL));
    }

    if (tick % STATUS_PERIOD_TICKS == 0) {
        publish_status("uptime=%llds modo=%s conexiones=%d fallos_creacion=%d wheel_state=%dHz",
                       (long long)(esp_timer_get_time() / 1000000), MODE_NAMES[s_mode], s_connections,
                       s_create_failures, state_rate_hz());
    }
}

static bool create_entities(void)
{
    s_allocator = rcl_get_default_allocator();

    rcl_init_options_t init_opts = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_opts, s_allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_opts, CONFIG_JONAS_ROS_DOMAIN_ID));
    RCCHECK(rclc_support_init_with_options(&s_support, 0, NULL, &init_opts, &s_allocator));
    RCSOFTCHECK(rcl_init_options_fini(&init_opts));

    RCCHECK(rclc_node_init_default(&s_node, "jonas_base", "", &s_support));

    const rosidl_message_type_support_t *ts_f32arr = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray);
    RCCHECK(rclc_subscription_init_default(&s_sub_wheel_vel, &s_node, ts_f32arr, "jonas/wheel_vel_cmd"));
    RCCHECK(rclc_subscription_init_default(&s_sub_wheel_pwm, &s_node, ts_f32arr, "jonas/wheel_pwm"));
    RCCHECK(rclc_subscription_init_default(&s_sub_gains, &s_node, ts_f32arr, "jonas/pid_gains"));
    RCCHECK(rclc_subscription_init_default(
        &s_sub_state_rate, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "jonas/wheel_state_rate"));

    RCCHECK(rclc_publisher_init_best_effort(
        &s_pub_ticks, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "jonas/wheel_ticks"));
    RCCHECK(rclc_publisher_init_best_effort(&s_pub_state, &s_node, ts_f32arr, "jonas/wheel_state"));
    RCCHECK(rclc_publisher_init_default(
        &s_pub_status, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "jonas/status"));
    RCCHECK(rclc_timer_init_default2(&s_pub_timer, &s_support, RCL_MS_TO_NS(PUB_PERIOD_MS), pub_timer_cb, true));

    s_executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&s_executor, &s_support.context, 5, &s_allocator));
    RCCHECK(rclc_executor_add_subscription(&s_executor, &s_sub_wheel_vel, &s_vel_msg, &wheel_vel_cb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&s_executor, &s_sub_wheel_pwm, &s_pwm_msg, &wheel_pwm_cb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&s_executor, &s_sub_gains, &s_gains_msg, &gains_cb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&s_executor, &s_sub_state_rate, &s_state_rate_msg, &state_rate_cb,
                                           ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&s_executor, &s_pub_timer));

    return true;
}

static void destroy_entities(void)
{
    rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&s_support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

    RCSOFTCHECK(rcl_publisher_fini(&s_pub_ticks, &s_node));
    RCSOFTCHECK(rcl_publisher_fini(&s_pub_state, &s_node));
    RCSOFTCHECK(rcl_publisher_fini(&s_pub_status, &s_node));
    RCSOFTCHECK(rcl_subscription_fini(&s_sub_wheel_vel, &s_node));
    RCSOFTCHECK(rcl_subscription_fini(&s_sub_wheel_pwm, &s_node));
    RCSOFTCHECK(rcl_subscription_fini(&s_sub_gains, &s_node));
    RCSOFTCHECK(rcl_subscription_fini(&s_sub_state_rate, &s_node));
    RCSOFTCHECK(rcl_timer_fini(&s_pub_timer));
    RCSOFTCHECK(rclc_executor_fini(&s_executor));
    RCSOFTCHECK(rcl_node_fini(&s_node));
    RCSOFTCHECK(rclc_support_fini(&s_support));
}

typedef enum { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } conn_state_t;

static void micro_ros_task(void *arg)
{
    conn_state_t state = WAITING_AGENT;
    int64_t last_ping_us = 0;

    while (1) {
        switch (state) {
        case WAITING_AGENT:
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                state = AGENT_AVAILABLE;
            } else {
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            break;

        case AGENT_AVAILABLE:
            if (create_entities()) {
                ESP_LOGI(TAG, "conectado al Agent");
                s_agent_connected = true;
                last_ping_us = esp_timer_get_time();
                state = AGENT_CONNECTED;
                publish_status("conectado: arranque por %s, uptime=%llds, motores=%d, conexiones=%d, "
                               "fallos_creacion=%d, dominio=%d",
                               reset_reason_str(esp_reset_reason()), (long long)(esp_timer_get_time() / 1000000),
                               JONAS_NUM_MOTORS, ++s_connections, s_create_failures, CONFIG_JONAS_ROS_DOMAIN_ID);
            } else {
                s_create_failures++;
                destroy_entities();
                state = WAITING_AGENT;
            }
            break;

        case AGENT_CONNECTED: {
            int64_t now = esp_timer_get_time();
            if (now - last_ping_us > 1000 * 1000) {
                last_ping_us = now;
                if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
                    state = AGENT_DISCONNECTED;
                    break;
                }
            }
            rclc_executor_spin_some(&s_executor, RCL_MS_TO_NS(5));
            vTaskDelay(1);
            break;
        }

        case AGENT_DISCONNECTED:
            ESP_LOGW(TAG, "Agent perdido, reintentando");
            s_agent_connected = false;
            destroy_entities();
            state = WAITING_AGENT;
            break;
        }
    }
}

// ---------------------------------------------------------------- app_main

static uart_port_t s_uart_port = UART_NUM_0;

void app_main(void)
{
    // Primero que nada: todas las salidas de motor en bajo (evita tirones al arrancar).
    for (int i = 0; i < NW; i++) {
        motor_force_low(JONAS_WHEEL_HW[i].pwm, JONAS_WHEEL_HW[i].dir);
    }

    for (int i = 0; i < JONAS_NUM_MOTORS; i++) {
        const jonas_wheel_hw_t *hw = &JONAS_WHEEL_HW[i];
        ESP_ERROR_CHECK(motor_init(i, hw->pwm, hw->dir, JONAS_PWM_FREQ_HZ, hw->invert_motor));
        ESP_ERROR_CHECK(encoder_init(i, hw->enc_a, hw->enc_b, hw->invert_encoder));
    }

    init_array_msg(&s_vel_msg, s_vel_buf, NW, 0);
    init_array_msg(&s_pwm_msg, s_pwm_buf, NW, 0);
    init_array_msg(&s_gains_msg, s_gains_buf, 4, 0);
    init_array_msg(&s_state_msg, s_state_buf, 3 * NW, 3 * NW);
    memset(&s_ticks_msg, 0, sizeof(s_ticks_msg));
    s_ticks_msg.data.data = s_ticks_buf;
    s_ticks_msg.data.capacity = 1 + NW;
    s_ticks_msg.data.size = 1 + NW;

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    rmw_uros_set_custom_transport(true, (void *)&s_uart_port, jonas_uart_open, jonas_uart_close,
                                  jonas_uart_write, jonas_uart_read);
#else
#error "micro-ROS debe compilarse con transporte custom (ver app-colcon.meta)"
#endif

    ESP_LOGI(TAG, "Jonas: %d motor(es), CPR=%.0f, control %d Hz, UART0 @ %d", JONAS_NUM_MOTORS,
             JONAS_ENCODER_CPR, CONFIG_JONAS_CONTROL_HZ, CONFIG_JONAS_UART_BAUD);

    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO + 1, NULL, 1);
    xTaskCreatePinnedToCore(micro_ros_task, "uros", CONFIG_MICRO_ROS_APP_STACK, NULL,
                            CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL, 0);
}
