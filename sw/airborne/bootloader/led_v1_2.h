#ifndef LED_H
#define LED_H
#include <avr/io.h>

#define LEDS_INIT() {				\
    DDRA |= _BV(7) | _BV(6) | _BV(5);			\
  }
#define RED_LED_ON()        PORTA &= ~_BV(5)
#define RED_LED_OFF()       PORTA |=  _BV(5)
#define RED_LED_TOGGLE()    PORTA ^=  _BV(5)

#define YELLOW_LED_ON()     PORTA &= ~_BV(7)
#define YELLOW_LED_OFF()    PORTA |=  _BV(7)
#define YELLOW_LED_TOGGLE() PORTA ^=  _BV(7)

#define GREEN_LED_ON()      PORTA &= ~_BV(6)
#define GREEN_LED_OFF()     PORTA |=  _BV(6)
#define GREEN_LED_TOGGLE()  PORTA ^=  _BV(6)




#endif /* LED_H */
