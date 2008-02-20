#ifndef MAX1167_HW_H
#define MAX1167_HW_H

/*
  MAX1167 SPI ADC connected on SPI1 
  SS on P1.29
  EOC on P0.16 ( EINT0 )
*/

#include "std.h"
#include "LPC21xx.h"
#include "interrupt_hw.h"  
#include "spi_hw.h"

#include "booz_debug.h"

#define MAX1167_ERR_ISR_STATUS   0
#define MAX1167_ERR_READ_OVERUN  1
#define MAX1167_ERR_SPURIOUS_EOC 2

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

#define Max1167OnSpiInt() {						\
    ASSERT((max1167_status == STA_MAX1167_READING_RES),			\
	   DEBUG_MAX_1117, MAX1167_ERR_ISR_STATUS);			\
    /* read dummy control byte reply */					\
    uint8_t foo __attribute__ ((unused));				\
    foo = SSPDR;							\
    /* store convertion result */					\
    max1167_values[0] = SSPDR << 8;					\
    max1167_values[0] += SSPDR;						\
    max1167_values[1] = SSPDR << 8;					\
    max1167_values[1] += SSPDR;						\
    max1167_values[2] = SSPDR << 8;					\
    max1167_values[2] += SSPDR;						\
    SpiClearRti();							\
    SpiDisableRti();							\
    SpiDisable();							\
    Max1167Unselect();							\
    max1167_status = STA_MAX1167_DATA_AVAILABLE;			\
  }

#endif /* MAX1167_WH */
