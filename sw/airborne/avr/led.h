/*
 * $Id$
 *  
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */
/** \file led.h
 *  \brief LED signaling
 *
 */
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
