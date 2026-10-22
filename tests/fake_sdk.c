/*
 * fake_sdk.c
 * Fake Pico SDK pieces for host tests of air530.c. The fake UART has the real
 * one's 32-byte receive FIFO; the "interrupt" runs inside fake_uart_receive()
 * each time the FIFO has data, the way the real one fires while bytes arrive.
 *
 * Author:  OCE360/560 instructors
 * License: MIT
 */
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "fake_sdk.h"

#define FIFO_SIZE 32
#define TX_CAPACITY 2048
#define PIN_COUNT 48
#define IRQ_COUNT 64

struct uart_inst { unsigned int index; };
static struct uart_inst fake_uart0 = {0}, fake_uart1 = {1};
uart_inst_t *uart0 = &fake_uart0;
uart_inst_t *uart1 = &fake_uart1;

static char fifo[FIFO_SIZE];
static size_t fifo_count;
static char tx_bytes[TX_CAPACITY];
static size_t tx_length;
static int pin_function[PIN_COUNT];
static unsigned int baud;
static bool rx_irq_on;
static irq_handler_t handlers[IRQ_COUNT];
static bool irq_on[IRQ_COUNT];
static uint64_t now_us;

void fake_sdk_reset(void) {
    fifo_count = 0;
    fake_uart_clear_sent();
    memset(pin_function, -1, sizeof pin_function);
    baud = 0;
    rx_irq_on = false;
    memset(handlers, 0, sizeof handlers);
    memset(irq_on, 0, sizeof irq_on);
}

void fake_uart_receive(const char *text) {
    for (const char *p = text; *p != '\0'; p++) {
        if (fifo_count < FIFO_SIZE) {
            fifo[fifo_count++] = *p;   // a full FIFO drops bytes, like the real one
        }
        if (rx_irq_on && handlers[UART1_IRQ] != NULL && irq_on[UART1_IRQ]) {
            handlers[UART1_IRQ]();
        }
    }
}

const char *fake_uart_sent(void) { return tx_bytes; }

void fake_uart_clear_sent(void) {
    tx_length = 0;
    tx_bytes[0] = '\0';
}

int fake_pin_function(unsigned int pin) { return pin < PIN_COUNT ? pin_function[pin] : -1; }
unsigned int fake_uart_baud(void) { return baud; }
bool fake_irq_ready(unsigned int irq) { return irq < IRQ_COUNT && handlers[irq] != NULL && irq_on[irq]; }

void gpio_set_function(unsigned int pin, int function) {
    if (pin < PIN_COUNT) pin_function[pin] = function;
}
void sleep_ms(uint32_t ms) { now_us += ms * 1000ull; }
uint64_t time_us_64(void) { return now_us += 10; }

void irq_set_exclusive_handler(unsigned int irq, irq_handler_t handler) {
    if (irq < IRQ_COUNT) handlers[irq] = handler;   // the real one panics if one is already set
}
void irq_remove_handler(unsigned int irq, irq_handler_t handler) {
    if (irq < IRQ_COUNT && handlers[irq] == handler) handlers[irq] = NULL;
}
void irq_set_enabled(unsigned int irq, bool enabled) {
    if (irq < IRQ_COUNT) irq_on[irq] = enabled;
}

unsigned int uart_init(uart_inst_t *uart, unsigned int baudrate) { (void)uart; baud = baudrate; return baud; }
unsigned int uart_set_baudrate(uart_inst_t *uart, unsigned int baudrate) { (void)uart; baud = baudrate; return baud; }
unsigned int uart_get_index(uart_inst_t *uart) { return uart->index; }
void uart_set_hw_flow(uart_inst_t *uart, bool cts, bool rts) { (void)uart; (void)cts; (void)rts; }
void uart_set_irq_enables(uart_inst_t *uart, bool rx_has_data, bool tx_needs_data) {
    (void)uart; (void)tx_needs_data;
    rx_irq_on = rx_has_data;
}
bool uart_is_readable(uart_inst_t *uart) { (void)uart; return fifo_count > 0; }
char uart_getc(uart_inst_t *uart) {
    (void)uart;
    char first = fifo[0];
    memmove(fifo, fifo + 1, --fifo_count);
    return first;
}
void uart_puts(uart_inst_t *uart, const char *text) {
    (void)uart;
    size_t length = strlen(text);
    if (tx_length + length < TX_CAPACITY) {
        memcpy(tx_bytes + tx_length, text, length + 1);
        tx_length += length;
    }
}
void uart_tx_wait_blocking(uart_inst_t *uart) { (void)uart; }
