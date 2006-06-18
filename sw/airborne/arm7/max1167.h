#ifndef MAX1167_H
#define MAX1167_H

/*
  MAX1167 SPI ADC connected on SPI1 
  SS on P1.29
  EOC on P0.16 ( EINT0 )
*/

#include "std.h"
#include "LPC21xx.h"
#include "interrupt_hw.h"  
#include "spi_hw.h"

#define MAX1167_NB_CHAN 4

extern void max1167_init( void );
extern void max1167_read( void );
extern volatile uint8_t max1167_data_available;
extern uint16_t max1167_values[MAX1167_NB_CHAN];

#define MAX1167_SS_PIN 29
#define MAX1167_SS_IODIR IO1DIR
#define MAX1167_SS_IOSET IO1SET
#define MAX1167_SS_IOCLR IO1CLR

#define MAX1167_EOC_PIN 16
#define MAX1167_EOC_PINSEL PINSEL1
#define MAX1167_EOC_PINSEL_BIT 0
#define MAX1167_EOC_PINSEL_VAL 1
#define MAX1167_EOC_EINT 0

#define Max1167Unselect() SetBit(MAX1167_SS_IOSET, MAX1167_SS_PIN)
#define Max1167Select() SetBit(MAX1167_SS_IOCLR, MAX1167_SS_PIN)

#define Max1167OnSpiInt() { \
    if (bit_is_set(SSPMIS, RTMIS)) {		\
      max1167_values[0] = SSPDR << 8;		\
      max1167_values[0] += SSPDR;		\
      max1167_values[1] = SSPDR << 8;		\
      max1167_values[1] += SSPDR;		\
      max1167_values[2] = SSPDR << 8;		\
      max1167_values[2] += SSPDR;		\
      max1167_data_available = TRUE;		\
      /* clear RTI */				\
      SpiClearRti();				\
      /* disable RTI */				\
      SpiDisableRti();				\
      /* disable SPI */				\
      SpiDisable();				\
      /* unselected max1167 */			\
      Max1167Unselect();			\
    }						\
  }

#endif /* MAX1167_H */
