
#include "LPC21xx.h"

#include "std.h"

#include "mcu.h"
#include "led.h"

#define TXD0_PIN 0
#define RXD0_PIN 1
#define TXD1_PIN 8
#define RXD1_PIN 9

int main (int argc, char** argv) {
  int tx=0, rx=0;
  int tx_shadow=1, rx_shadow=1;
  mcu_init();
  led_init();
  LED_ON(1);

  /* TXD0 and TXD1 output */
  SetBit(IO0DIR, TXD0_PIN);
  SetBit(IO0DIR, TXD1_PIN);

  /* RXD0 and RXD1 input */
  ClearBit(IO0DIR,RXD0_PIN);
  ClearBit(IO0DIR,RXD1_PIN);

  /* use shadow bits to reduce jitter */
  while(1) {
    tx = bit_is_set(IO0PIN, RXD0_PIN);
    if (tx != tx_shadow) {
      if (tx) {
        SetBit(IO0SET, TXD1_PIN);
      } else {
        SetBit(IO0CLR, TXD1_PIN);
      }
      tx_shadow = tx;
      LED_TOGGLE(2);
    }
    rx = bit_is_set(IO0PIN, RXD1_PIN);
    if (rx != rx_shadow) {
      if (rx) {
        SetBit(IO0SET, TXD0_PIN);
      } else {
        SetBit(IO0CLR, TXD0_PIN);
      }
      rx_shadow = rx;
      LED_TOGGLE(3);
    }
  }
  return 0;
}
