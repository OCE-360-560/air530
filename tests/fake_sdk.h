/*
 * fake_sdk.h
 * A fake UART (with its receive interrupt), GPIO, IRQ and clock for host tests
 * of air530.c: queue bytes as if the GPS sent them, read back what the driver wrote.
 *
 * Author:  OCE360/560 instructors
 * License: MIT
 */
#ifndef FAKE_SDK_H
#define FAKE_SDK_H

#include <stdbool.h>
#include <stddef.h>

void fake_sdk_reset(void);
/** Bytes arrive at the UART; the receive interrupt runs if the driver turned it on. */
void fake_uart_receive(const char *text);
/** Everything written with uart_puts() since the last fake_sdk_reset() or fake_uart_clear_sent(). */
const char *fake_uart_sent(void);
void fake_uart_clear_sent(void);
int fake_pin_function(unsigned int pin);
unsigned int fake_uart_baud(void);
/** True if a handler is installed and enabled for irq. */
bool fake_irq_ready(unsigned int irq);

#endif
