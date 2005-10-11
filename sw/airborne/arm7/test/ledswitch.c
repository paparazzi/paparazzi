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

// olimex LPC-P2138: buttons on P0.15/P0.16 (active low)
#define BUT1PIN 	15
#define BUT2PIN 	16
// olimex LPC-P2138: LEDs on P0.12/P0.13 (active low)
#define LED1PIN  	12
#define LED2PIN  	13

static void delay(void )
{
  volatile int i,j;
  for (i=0;i<100;i++)
    for (j=0;j<1000;j++);
}
 
#define YELLOW_LED_ON() { IOCLR0 = (1<<LED1PIN); } 
#define YELLOW_LED_OFF() { IOSET0 = (1<<LED1PIN); } // LEDs active low
#define GREEN_LED_ON() { IOCLR0 = (1<<LED2PIN); }
#define GREEN_LED_OFF() { IOSET0 = (1<<LED2PIN); }

// true if button released (active low)
#define BUTTTON1_OFF() (IOPIN0 & (1<<BUT1PIN))
#define BUTTTON2_OFF() (IOPIN0 & (1<<BUT2PIN))

int main(void)
{

  int i;

  MAMCR = 2;	// MAM functions fully enabled

  IODIR0 |= (1<<LED1PIN)|(1<<LED2PIN); // define LED-Pins as outputs
  YELLOW_LED_OFF();
  GREEN_LED_OFF();
  IODIR0 &= ~((1<<BUT1PIN)|(1<<BUT2PIN));// define Button-Pins as inputs

  i=0;
  while (i<10)	{
    YELLOW_LED_ON();
    GREEN_LED_OFF();
    delay();
    YELLOW_LED_OFF();
    GREEN_LED_ON();
    delay();
    i++;
  }
	
  while (1)	
    {
      if (BUTTTON1_OFF()) { 
	YELLOW_LED_ON();
      }
      else {
	YELLOW_LED_OFF();
      }
		
      if (BUTTTON2_OFF())	{ 
	GREEN_LED_ON();
      }
      else {
	GREEN_LED_OFF();
      }
    }
	
  return 0; // never reached
}
