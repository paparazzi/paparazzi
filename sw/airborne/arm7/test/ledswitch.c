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
  
int main(void)
{

  int i;

  MAMCR = 2;	// MAM functions fully enabled

  IODIR0 |= (1<<LED1PIN)|(1<<LED2PIN); // define LED-Pins as outputs
  IOSET0 = (1<<LED1PIN)|(1<<LED2PIN); // set Bits = LEDs off (active low)
  IODIR0 &= ~((1<<BUT1PIN)|(1<<BUT2PIN));// define Button-Pins as inputs

  i=0;
  while (i<10)	
    {
      IOCLR0 = (1<<LED1PIN);	
      IOSET0 = (1<<LED2PIN);	
      delay();
      IOSET0 = (1<<LED1PIN);
      IOCLR0 = (1<<LED2PIN);	
      delay();
      i++;
    }
	
  while (1)	
    {
      if (IOPIN0 & (1<<BUT1PIN))	{ // true if button released (active low)
	IOCLR0 = (1<<LED2PIN);		// clear I/O bit -> LED on (active low)
      }
      else {
	IOSET0 = (1<<LED2PIN);		// set I/O bit -> LED off (active low)
      }
		
      if (IOPIN0 & (1<<BUT2PIN))	{ // true if button released (active low)
	IOCLR0 = (1<<LED1PIN);		// clear I/O bit -> LED on (active low)
      }
      else {
	IOSET0 = (1<<LED1PIN);		// set I/O bit -> LED off (active low)
      }
    }
	
  return 0; // never reached
}
