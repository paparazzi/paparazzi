#include "led_v1_2.h"

int main ( void ) {
  LEDS_INIT();
  RED_LED_OFF();
  YELLOW_LED_OFF();
  GREEN_LED_OFF();
  while(1) {
    uint16_t foo;
    while (foo--);
    GREEN_LED_TOGGLE();
  }
  return 0;
}
