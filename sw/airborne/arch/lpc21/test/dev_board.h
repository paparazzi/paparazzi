#ifndef DEV_BOARD_H
#define DEV_BOARD_H

// olimex LPC-P2138:

#define FOSC 14745600
#define PLL_MUL 4

#if 1 // DEV_BOARD_TYPE
// OLIMEX

// LEDs on P0.12/P0.13 (active low)
#define LED1PIN   12
#define LED2PIN   13

// define LED-Pins as outputs
#define LED_INIT() {  IODIR0 |= (1<<LED1PIN)|(1<<LED2PIN); }
#define YELLOW_LED_ON() { IOCLR0 = (1<<LED1PIN); }
#define YELLOW_LED_OFF() { IOSET0 = (1<<LED1PIN); } // LEDs active low
#define GREEN_LED_ON() { IOCLR0 = (1<<LED2PIN); }
#define GREEN_LED_OFF() { IOSET0 = (1<<LED2PIN); }

#else
// KEIL

// LEDs on P1.16/P1.17 (active high)
#define LED1PIN   16
#define LED2PIN   17

// define LED-Pins as outputs
#define LED_INIT() {  IODIR1 |= (1<<LED1PIN)|(1<<LED2PIN); }
#define YELLOW_LED_OFF() { IOCLR1 = (1<<LED1PIN); }
#define YELLOW_LED_ON() { IOSET1 = (1<<LED1PIN); } // LEDs active low
#define GREEN_LED_OFF() { IOCLR1 = (1<<LED2PIN); }
#define GREEN_LED_ON() { IOSET1 = (1<<LED2PIN); }

#endif  // DEV_BOARD_TYPE

// buttons on P0.15/P0.16 (active low)
#define BUT1PIN   15
#define BUT2PIN   16

// define Button-Pins as inputs
#define BUTTON_INIT() {  IODIR0 &= ~((1<<BUT1PIN)|(1<<BUT2PIN)); }
// true if button released (active low)
#define BUTTTON1_OFF() (IOPIN0 & (1<<BUT1PIN))
#define BUTTTON2_OFF() (IOPIN0 & (1<<BUT2PIN))

#endif /* DEV_BOARD_H */
