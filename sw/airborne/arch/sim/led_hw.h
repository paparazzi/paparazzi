#ifndef LED_HW_H
#define LED_HW_H

#include <stdio.h>
#include <caml/mlvalues.h>
#include <caml/memory.h>
#include <caml/callback.h>

extern value *leds_closure;
extern bool led_disable;

#define LED_INIT(i) { led_disable = false; }
#define LED_ON(i) { if (leds_closure && !led_disable) callback2(*leds_closure, Val_int(i), Val_int(1)); }
#define LED_OFF(i) { if (leds_closure && !led_disable) callback2(*leds_closure, Val_int(i), Val_int(0)); }
#define LED_TOGGLE(i) { if (leds_closure && !led_disable) callback2(*leds_closure, Val_int(i), Val_int(2)); }
#define LED_DISABLE(i) { LED_OFF(i); led_disable = true; }

#define LED_PERIODIC() {}

#endif /* LED_HW_H */
