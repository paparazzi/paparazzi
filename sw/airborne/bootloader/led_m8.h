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




#endif /* LED_H */
