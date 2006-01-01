#ifndef LED_HW_H
#define LED_HW_H

#include CONFIG
#include <avr/io.h>

#define _LED_PORT(p) PORT ## p
#define LED_PORT(p) _LED_PORT(p)
#define _LED_DDR(p)  DDR ## p
#define LED_DDR(p)  _LED_DDR(p)

#define LED_X_DDR(x) LED_DDR(LED_ ## x ## _BANK)
#define LED_X_PORT(x) LED_PORT(LED_ ## x ## _BANK)
#define LED_X_PIN(x) LED_ ## x ## _PIN

/* set pin as output */
#define LED_X_INIT(i)  LED_X_DDR(i) |= _BV(LED_X_PIN(i))

static inline void led_init ( void ) {
#ifdef LED_1_BANK
  LED_X_INIT(1);
#endif /* LED_1_BANK */
#ifdef LED_2_BANK
  LED_X_INIT(2);
#endif /* LED_2_BANK */
#ifdef LED_3_BANK
  LED_X_INIT(3);
#endif /* LED_3_BANK */
}

#define LED_ON(i) LED_X_PORT(i)  &= ~_BV(LED_X_PIN(i))
#define LED_OFF(i)     LED_X_PORT(i) |=  _BV(LED_X_PIN(i))
#define LED_TOGGLE(i)  LED_X_PORT(i) ^=  _BV(LED_X_PIN(i))

#endif /* LED_HW_H */
