#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <uxr/client/transport.h>

#ifdef __cplusplus
extern "C" {
#endif

// Transporte custom de micro-ROS sobre UART. transport->args debe apuntar a un uart_port_t.
bool   jonas_uart_open(struct uxrCustomTransport *transport);
bool   jonas_uart_close(struct uxrCustomTransport *transport);
size_t jonas_uart_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t jonas_uart_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

#ifdef __cplusplus
}
#endif
