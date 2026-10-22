/* Host-test stand-in for hardware/irq.h. Fakes are in fake_sdk.c. */
#ifndef FAKE_HARDWARE_IRQ_H
#define FAKE_HARDWARE_IRQ_H

#include <stdbool.h>

#define UART0_IRQ 33   // RP2350 numbers
#define UART1_IRQ 34

typedef void (*irq_handler_t)(void);
void irq_set_exclusive_handler(unsigned int irq, irq_handler_t handler);
void irq_remove_handler(unsigned int irq, irq_handler_t handler);
void irq_set_enabled(unsigned int irq, bool enabled);

#endif
