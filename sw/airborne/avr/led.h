/* 3 leds plugged on rc receiver port for debug purpose */

#ifndef LED_H
#define LED_H
#include <avr/io.h>

#define LEDS_INIT() {				\
    DDRB |= _BV(0) | _BV(1);			\
    DDRC |= _BV(0);				\
  }
#define RED_LED_ON()        PORTC &= ~_BV(0)
#define RED_LED_OFF()       PORTC |=  _BV(0)
#define RED_LED_TOGGLE()    PORTC ^=  _BV(0)

#define YELLOW_LED_ON()     PORTB &= ~_BV(0)
#define YELLOW_LED_OFF()    PORTB |=  _BV(0)
#define YELLOW_LED_TOGGLE() PORTB ^=  _BV(0)

#define GREEN_LED_ON()      PORTB &= ~_BV(1)
#define GREEN_LED_OFF()     PORTB |=  _BV(1)
#define GREEN_LED_TOGGLE()  PORTB ^=  _BV(1)

#define CounterPin 4

#define CounterLedInit()  DDRD |= _BV(CounterPin)

#define CounterLedOn()      PORTD &= ~_BV(CounterPin)
#define CounterLedOff()     PORTD |=  _BV(CounterPin)
#define CounterLedToggle()  PORTD ^=  _BV(CounterPin)





#endif /* LED_H */
