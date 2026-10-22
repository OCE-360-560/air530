/* Host-test stand-in for hardware/uart.h. Fakes are in fake_sdk.c. */
#ifndef FAKE_HARDWARE_UART_H
#define FAKE_HARDWARE_UART_H

#include <stdbool.h>
#include <stdint.h>

typedef struct uart_inst uart_inst_t;
extern uart_inst_t *uart0;
extern uart_inst_t *uart1;

unsigned int uart_init(uart_inst_t *uart, unsigned int baudrate);
unsigned int uart_set_baudrate(uart_inst_t *uart, unsigned int baudrate);
unsigned int uart_get_index(uart_inst_t *uart);
void uart_set_hw_flow(uart_inst_t *uart, bool cts, bool rts);
void uart_set_irq_enables(uart_inst_t *uart, bool rx_has_data, bool tx_needs_data);
bool uart_is_readable(uart_inst_t *uart);
char uart_getc(uart_inst_t *uart);
void uart_puts(uart_inst_t *uart, const char *text);
void uart_tx_wait_blocking(uart_inst_t *uart);

#endif
