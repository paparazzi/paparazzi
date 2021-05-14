#ifndef LED_HW_H
#define LED_HW_H

#include <stdbool.h>

extern bool led_disabled;
extern void _led_on(int i);
extern void _led_off(int i);
extern void _led_toggle(int i);

#define LED_INIT(i) { led_disabled = false; }
#define LED_ON(i) _led_on(i);
#define LED_OFF(i) _led_off(i);
#define LED_TOGGLE(i) _led_toggle(i);
#define LED_DISABLE(i) { LED_OFF(i); led_disabled = true; }

#define LED_PERIODIC() {}

#endif /* LED_HW_H */
