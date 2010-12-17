#ifndef CONFIG_V1_2_1_H
#define CONFIG_V1_2_1_H

#include "twin_avr.h"

/* clock in MHz */
#define CLOCK 16

/* PPM input on mega8 ICP PB0 */
#define PPM_DDR  DDRB
#define PPM_PORT PORTB
#define PPM_PIN  PB0

/* 4017 servos */
#define _4017_RESET_PORT        PORTD
#define _4017_RESET_DDR         DDRD
#define _4017_RESET_PIN         7

#define _4017_CLOCK_PORT        PORTB
#define _4017_CLOCK_DDR         DDRB
#define _4017_CLOCK_PIN         PB1

#define SERVO_OCR		OCR1A
#define SERVO_ENABLE		OCIE1A
#define SERVO_FLAG		OCF1A
#define SERVO_FORCE		FOC1A
#define SERVO_COM0		COM1A0
#define SERVO_COM1		COM1A1

#define ADC_CHANNEL_VSUPPLY     3

#endif /* CONFIG_V1_2_1_H */
