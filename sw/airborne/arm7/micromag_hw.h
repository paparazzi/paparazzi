#ifndef MICROMAG_HW_H
#define MICROMAG_HW_H

#include "std.h"
#include "LPC21xx.h"
#include "interrupt_hw.h" 
#include "spi_hw.h"

#include "airframe.h"

extern volatile uint8_t micromag_cur_axe;


#define MmSelect() SetBit(MM_SS_IOCLR,MM_SS_PIN)
#define MmUnselect() SetBit(MM_SS_IOSET,MM_SS_PIN)

#define MmReset() SetBit(MM_RESET_IOCLR,MM_RESET_PIN)
#define MmSet() SetBit(MM_RESET_IOSET,MM_RESET_PIN)

#define MmTriggerRead() {					\
    MmSet();							\
    MmReset();							\
    uint8_t control_byte = (micromag_cur_axe+1) << 0 | 4 << 4;	\
    SSPDR = control_byte; /* SpiSend(control_byte);	*/	\
  }

#if 0
#define MmOnSpiIt() {					\
    if (bit_is_set(SSPMIS, RTMIS)) {			\
      micromag_values[micromag_cur_axe] = SSPDR << 8;	\
      micromag_values[micromag_cur_axe] += SSPDR;	\
      /* clear RTI */					\
      SpiClearRti();					\
      /* disable RTI */					\
      SpiDisableRti();					\
      micromag_data_available = TRUE;			\
      /* disable SPI */					\
      SpiDisable();					\
      /* unselected device */				\
      MmUnselect();					\
    }							\
}
#endif

#define MicromagOnSpiInt() {				\
    if (bit_is_set(SSPMIS, RTMIS)) {			\
      micromag_values[micromag_cur_axe] = SSPDR << 8;	\
      micromag_values[micromag_cur_axe] += SSPDR;	\
      SpiClearRti();					\
      SpiDisableRti();					\
      if (micromag_cur_axe == 2) {			\
	micromag_status = MM_DATA_AVAILABLE;		\
	SpiDisable();					\
	MmUnselect();					\
      }							\
      else {						\
	micromag_cur_axe++;				\
	MmTriggerRead();				\
      }							\
    }							\
  }

#endif /* MICROMAG_HW_H */
