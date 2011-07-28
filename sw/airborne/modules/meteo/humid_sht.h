#ifndef HUMID_SHT_H
#define HUMID_SHT_H

#include "std.h"

#ifdef STM32
#error LPC21_only
#endif

/* GPIO P0.x defaults */
#ifndef DAT_PIN
/* ADC1 Port, ADC_4, P0.30 */
#define DAT_PIN         30
/* IRH Port, IRH_2, P0.25 */
// #define DAT_PIN         25
#endif

#ifndef SCK_PIN
/* ADC1 Port, ADC_3, P0.4 */
#define SCK_PIN         4
/* IRH Port, IRH_1, P0.22 */
// #define SCK_PIN         22
#endif

#define noACK           0
#define ACK             1
#define TEMP            0
#define HUMI            1

//adr    command  r/w
//000    0011     0
#define STATUS_REG_W    0x06
//000    0011     1
#define STATUS_REG_R    0x07
//000    0001     1
#define MEASURE_TEMP    0x03
//000    0010     1
#define MEASURE_HUMI    0x05
//000    1111     0
#define RESET           0x1e

// set data pin to input and set to low
#define DATA_INIT       (IO0DIR &= ~(_BV(DAT_PIN)); IO0CLR=_BV(DAT_PIN))
// set data pin to input
#define DATA_SET        (IO0DIR &= ~(_BV(DAT_PIN)))
// set data pin to output
#define DATA_CLR        (IO0DIR |= _BV(DAT_PIN))
// get data pin
#define DATA_IN         ((IO0PIN & _BV(DAT_PIN))>>DAT_PIN)

// set clock pin to output and set to low
#define SCK_INIT        (IO0DIR=_BV(SCK_PIN); IO0CLR=_BV(SCK_PIN))
// set clock pin to high
#define SCK_SET         (IO0SET=_BV(SCK_PIN))
// set clock pin to low
#define SCK_CLR         (IO0CLR=_BV(SCK_PIN))

#define SHT_IDLE            0
#define SHT_MEASURING_HUMID 1
#define SHT_MEASURING_TEMP  2

extern uint16_t humidsht, tempsht;
extern float fhumidsht, ftempsht;
extern bool_t humid_sht_available;
extern uint8_t humid_sht_status;

void humid_sht_init( void );
void humid_sht_periodic(void);

#endif /* HUMID_SHT_H */
