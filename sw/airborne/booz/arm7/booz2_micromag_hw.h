#ifndef BOOZ2_MICROMAG_HW_H
#define BOOZ2_MICROMAG_HW_H

#include <stdlib.h>  // for abs

#include "std.h"
#include "LPC21xx.h"
#include "interrupt_hw.h" 
#include "spi_hw.h"

#include "airframe.h"
#include "led.h"

extern volatile uint8_t booz2_micromag_cur_axe;

#define MmSelect() SetBit(MM_SS_IOCLR,MM_SS_PIN)
#define MmUnselect() SetBit(MM_SS_IOSET,MM_SS_PIN)

#define MmReset() SetBit(MM_RESET_IOCLR,MM_RESET_PIN)
#define MmSet() SetBit(MM_RESET_IOSET,MM_RESET_PIN)

#define MmOnSpiIt() {							\
    switch (booz2_micromag_status) {						\
    case MM_SENDING_REQ:						\
      {									\
	/* read dummy control byte reply */				\
	uint8_t foo __attribute__ ((unused)) = SSPDR;			\
	booz2_micromag_status = MM_WAITING_EOC;				\
	MmUnselect();							\
	SpiClearRti();							\
	SpiDisableRti();						\
	SpiDisable();							\
      }									\
      break;								\
    case MM_READING_RES:						\
      {									\
	int16_t new_val;						\
	new_val = SSPDR << 8;						\
	new_val += SSPDR;						\
	if (abs(new_val) < 2000)					\
	  booz2_micromag_values[booz2_micromag_cur_axe] = new_val;			\
	MmUnselect();							\
	SpiClearRti();							\
	SpiDisableRti();						\
	SpiDisable();							\
	booz2_micromag_cur_axe++;						\
	if (booz2_micromag_cur_axe > 2) {					\
	  booz2_micromag_cur_axe = 0;						\
	  booz2_micromag_status = MM_DATA_AVAILABLE;				\
	}								\
	else								\
	  booz2_micromag_status = MM_IDLE;					\
      }									\
      break;								\
    }									\
  }


#define MmSendReq() {							\
    MmSelect();								\
    booz2_micromag_status = MM_SENDING_REQ;					\
    MmSet();								\
    SpiClearRti();							\
    SpiEnableRti();							\
    uint8_t control_byte = (booz2_micromag_cur_axe+1) << 0 | 3 << 4;		\
    SSPDR = control_byte;						\
    MmReset();								\
    SpiEnable();                                                        \
  }

#define MmReadRes() {							\
    booz2_micromag_status = MM_READING_RES;					\
    MmSelect();								\
    SpiClearRti();							\
    SpiEnableRti();							\
    /* trigger 2 bytes read */						\
    SSPDR = 0;								\
    SSPDR = 0;								\
    SpiEnable();                                                        \
  }



#endif /* MICROMAG_HW_H */
