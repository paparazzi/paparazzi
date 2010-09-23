#ifndef LED_HW_H
#define LED_HW_H

#include CONFIG
#include <avr/io.h>

#define __LED_PORT(p) PORT ## p
#define _LED_PORT(p) __LED_PORT(p)
#define __LED_DDR(p) DDR ## p
#define _LED_DDR(p) __LED_DDR(p)

#define LED_DDR(x) _LED_DDR(LED_ ## x ## _BANK)
#define LED_PORT(x) _LED_PORT(LED_ ## x ## _BANK)
#define LED_PIN(x) LED_ ## x ## _PIN

/* set pin as output */
#define LED_INIT(i)  LED_DDR(i) |= _BV(LED_PIN(i))

#define LED_ON(i) LED_PORT(i)  &= ~_BV(LED_PIN(i))
#define LED_OFF(i) LED_PORT(i) |=  _BV(LED_PIN(i))
#define LED_TOGGLE(i) LED_PORT(i) ^=  _BV(LED_PIN(i))

#endif /* LED_HW_H */
