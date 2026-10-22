/* Host-test stand-in for the Pico SDK: just what air530.c uses. Fakes are in fake_sdk.c. */
#ifndef FAKE_PICO_STDLIB_H
#define FAKE_PICO_STDLIB_H

#include <stdbool.h>
#include <stdint.h>

#define GPIO_FUNC_UART 2
void gpio_set_function(unsigned int pin, int function);
void sleep_ms(uint32_t ms);
uint64_t time_us_64(void);

#endif
