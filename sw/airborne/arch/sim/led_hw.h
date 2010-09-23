#include <stdio.h>
#include <caml/mlvalues.h>
#include <caml/memory.h>
#include <caml/callback.h>

extern value * leds_closure;

#define LED_INIT(i) { }
#define LED_ON(i) { if (leds_closure) callback2(*leds_closure, Val_int(i), Val_int(1)); }
#define LED_OFF(i) { if (leds_closure) callback2(*leds_closure, Val_int(i), Val_int(0)); }
#define LED_TOGGLE(i) { if (leds_closure) callback2(*leds_closure, Val_int(i), Val_int(2)); }
 
