#include "led_hw.h"

#include <stdio.h>
#include <caml/mlvalues.h>
#include <caml/memory.h>
#include <caml/callback.h>

value *leds_closure = 0;
bool led_disabled = false;

value register_leds_cb(value cb_name)
{
  leds_closure = caml_named_value(String_val(cb_name));
  return Val_unit;
}

void _led_on(int i) { if (leds_closure && !led_disabled) callback2(*leds_closure, Val_int(i), Val_int(1)); }
void _led_off(int i) { if (leds_closure && !led_disabled) callback2(*leds_closure, Val_int(i), Val_int(0)); }
void _led_toggle(int i) { if (leds_closure && !led_disabled) callback2(*leds_closure, Val_int(i), Val_int(2)); }

