#include "uart_transport.h"

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

#define UART_BUF_SIZE 1024

bool jonas_uart_open(struct uxrCustomTransport *transport)
{
    uart_port_t port = *(uart_port_t *)transport->args;

    // Tras una reconexión el driver puede seguir instalado.
    if (uart_is_driver_installed(port)) {
        uart_driver_delete(port);
    }

    const uart_config_t cfg = {
        .baud_rate = CONFIG_JONAS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (uart_driver_install(port, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 0, NULL, 0) != ESP_OK) {
        return false;
    }
    if (uart_param_config(port, &cfg) != ESP_OK) {
        return false;
    }
    // -1 = pines por defecto de UART0 en el S3 (TX=GPIO43, RX=GPIO44), cableados al CH343.
    if (uart_set_pin(port, CONFIG_MICROROS_UART_TXD, CONFIG_MICROROS_UART_RXD,
                     CONFIG_MICROROS_UART_RTS, CONFIG_MICROROS_UART_CTS) != ESP_OK) {
        return false;
    }
    uart_flush_input(port);
    return true;
}

bool jonas_uart_close(struct uxrCustomTransport *transport)
{
    uart_port_t port = *(uart_port_t *)transport->args;
    return uart_driver_delete(port) == ESP_OK;
}

size_t jonas_uart_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err)
{
    (void)err;
    uart_port_t port = *(uart_port_t *)transport->args;
    int n = uart_write_bytes(port, (const char *)buf, len);
    return n < 0 ? 0 : (size_t)n;
}

size_t jonas_uart_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    (void)err;
    uart_port_t port = *(uart_port_t *)transport->args;
    int n = uart_read_bytes(port, buf, len, pdMS_TO_TICKS(timeout));
    return n < 0 ? 0 : (size_t)n;
}
