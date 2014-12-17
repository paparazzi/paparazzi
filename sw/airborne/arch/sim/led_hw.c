#include "led_hw.h"

value *leds_closure = 0;

value register_leds_cb(value cb_name)
{
  leds_closure = caml_named_value(String_val(cb_name));
  return Val_unit;
}
