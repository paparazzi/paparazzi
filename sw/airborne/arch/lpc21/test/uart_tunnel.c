
#include "lpc2138.h"

#define TXD0_PIN 0
#define RXD0_PIN 1
#define TXD1_PIN 8
#define RXD1_PIN 9

int main(int argc, char **argv)
{
  /* TXD0 and TXD1 output */
  IODIR0 |= (1 << TXD0_PIN) | (1 << TXD1_PIN);
  /* RXD0 and RXD1 input */
  IODIR0 &= ~((1 << RXD0_PIN) | (1 << RXD1_PIN));

  while (1) {
    if (IOPIN0 & (1 << RXD0_PIN)) {
      IOSET0 = (1 << TXD1_PIN);
    } else {
      IOCLR0 = (1 << TXD1_PIN);
    }
    if (IOPIN0 & (1 << RXD1_PIN)) {
      IOSET0 = (1 << TXD0_PIN);
    } else {
      IOCLR0 = (1 << TXD0_PIN);
    }
  }
  return 0;
}
