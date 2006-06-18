#ifndef MICROMAG_H
#define MICROMAG_H

#include "std.h"
#include "LPC21xx.h"
#include "interrupt_hw.h" 
#include "spi_hw.h"

#define MM_NB_AXIS 3

extern void micromag_init( void );
extern void micromag_read( void );

extern volatile uint8_t micromag_data_available;
extern int16_t micromag_values[MM_NB_AXIS];
extern volatile uint8_t micromag_cur_axe;

#define MM_SS_PIN 20
#define MM_SS_IODIR IO0DIR
#define MM_SS_IOSET IO0SET
#define MM_SS_IOCLR IO0CLR

#define MM_RESET_PIN 21
#define MM_RESET_IODIR IO1DIR
#define MM_RESET_IOSET IO1SET
#define MM_RESET_IOCLR IO1CLR

#define MM_DRDY_PIN 15
#define MM_DRDY_PINSEL PINSEL0
#define MM_DRDY_PINSEL_BIT 30
#define MM_DRDY_PINSEL_VAL 2
#define MM_DRDY_EINT 2

#define MmSelect() SetBit(MM_SS_IOCLR,MM_SS_PIN)
#define MmUnselect() SetBit(MM_SS_IOSET,MM_SS_PIN)

#define MmReset() SetBit(MM_RESET_IOCLR,MM_RESET_PIN)
#define MmSet() SetBit(MM_RESET_IOSET,MM_RESET_PIN)

#define MmTriggerRead() {					\
    MmSet();							\
    MmReset();							\
    uint8_t control_byte = (micromag_cur_axe+1) << 0 | 4 << 4;	\
    SpiSend(control_byte);					\
  }

#define MmOnSpiIt() { \
    if (bit_is_set(SSPMIS, RTMIS)) {			\
      micromag_values[micromag_cur_axe] = SSPDR << 8;	\
      micromag_values[micromag_cur_axe] += SSPDR;	\
      /* clear RTI */					\
      SpiClearRti();					\
      /* disable RTI */					\
      SpiDisableRti();					\
      micromag_cur_axe++;				\
      if (micromag_cur_axe > MM_NB_AXIS) {		\
	micromag_data_available = TRUE;			\
	/* disable SPI */				\
	SpiDisable();					\
	/* unselected max1167 */			\
	MmUnselect();					\
      }							\
      else {						\
	MmTriggerRead();				\
      }							\
    }							\
}


#endif /* MICROMAG_H */
