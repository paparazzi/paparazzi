//////////////////////////////////////////////////////////////////////////////
//
// Philips LPC2129 ARM7TDMI LED/Switch Example
//
// This example demonstrates writing to and reading from
// the GPIO port.
// (1) flash the LED 10 times
// (2) wait for key-press, turn off LED if key is pressed
//
// WinARM example by Martin THOMAS, Kaiserslautern, Germany
// (eversmith@heizung-thomas.de)
// http://www.siwawi.arubi.uni-kl.de/avr_projects
//////////////////////////////////////////////////////////////////////////////

#include "lpc21xx.h"
#include "lpc2138.h"

#include "dev_board.h"

static void delay(void)
{
  volatile int i, j;
  for (i = 0; i < 100; i++)
    for (j = 0; j < 1000; j++);
}

int main(void)
{

  int i;

  MAMCR = 2;  // MAM functions fully enabled

  LED_INIT();
  YELLOW_LED_OFF();
  GREEN_LED_OFF();
  BUTTON_INIT();
  i = 0;
  while (i < 10)  {
    YELLOW_LED_ON();
    GREEN_LED_OFF();
    delay();
    YELLOW_LED_OFF();
    GREEN_LED_ON();
    delay();
    i++;
  }

  while (1) {
    if (BUTTTON1_OFF()) {
      YELLOW_LED_ON();
    } else {
      YELLOW_LED_OFF();
    }

    if (BUTTTON2_OFF()) {
      GREEN_LED_ON();
    } else {
      GREEN_LED_OFF();
    }
  }

  return 0; // never reached
}
