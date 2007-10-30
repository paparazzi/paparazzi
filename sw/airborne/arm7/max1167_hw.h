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
      switch (max1167_status) {			\
						\
      case STA_MAX1167_IDLE:			\
	/* should not happen */			\
	break;					\
						\
      case STA_MAX1167_SENDING_REQ: {		\
	/* read dummy control byte reply */     \
        uint8_t foo __attribute__ ((unused));   \
	foo = SSPDR;				\
	SpiClearRti();				\
	SpiDisableRti();			\
	SpiDisable();				\
        /* Max1167Unselect(); maybe... */       \
        /* here we might enable extint0 */      \
        max1167_status = STA_MAX1167_WAIT_EOC;  \
      }						\
	break;					\
						\
      case STA_MAX1167_WAIT_EOC:		\
        /* should not happen */			\
        break;					\
						\
      case STA_MAX1167_READING_RES:		\
        /* store convertion result */           \
        max1167_values[0] = SSPDR << 8;		\
        max1167_values[0] += SSPDR;		\
        max1167_values[1] = SSPDR << 8;		\
        max1167_values[1] += SSPDR;		\
        max1167_values[2] = SSPDR << 8;		\
        max1167_values[2] += SSPDR;		\
        SpiClearRti();				\
        SpiDisableRti();			\
        SpiDisable();				\
        Max1167Unselect();			\
        max1167_status = STA_MAX1167_DATA_AVAILABLE; \
        break;					\
      case STA_MAX1167_DATA_AVAILABLE :		\
	/* should not happen */			\
        break;					\
    }						\
  }						\
}

#endif /* MAX1167_WH */
