
#include "LPC21xx.h"

#include "std.h"

#include "init_hw.h"
#include "led.h"

#define TXD0_PIN 0
#define RXD0_PIN 1
#define TXD1_PIN 8
#define RXD1_PIN 9

int main (int argc, char** argv) {
  hw_init();
  led_init();

  /* TXD0 and TXD1 output */
  SetBit(IO0DIR, TXD0_PIN);
  SetBit(IO0DIR, TXD1_PIN);

  /* RXD0 and RXD1 input */
  ClearBit(IO0DIR,RXD0_PIN);
  ClearBit(IO0DIR,RXD1_PIN);

  while(1) {
    if (bit_is_set(IO0PIN,RXD0_PIN))
      SetBit(IO0SET, TXD1_PIN);
    else
      SetBit(IO0CLR, TXD1_PIN);
    if (bit_is_set(IO0PIN, RXD1_PIN)) {
      LED_OFF(2);
      SetBit(IO0SET, TXD0_PIN);
    } else {
      LED_ON(2);
      SetBit(IO0CLR, TXD0_PIN);
    }
  }
  return 0;
}
