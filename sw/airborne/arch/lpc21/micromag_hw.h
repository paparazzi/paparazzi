#ifndef MICROMAG_HW_H
#define MICROMAG_HW_H

#include <stdlib.h>  // for abs

#include "std.h"
#include "LPC21xx.h"
#include "interrupt_hw.h"

#include "ssp_hw.h"
#include BOARD_CONFIG

#include "generated/airframe.h"

#define MM_DIVISOR_128  2
#define MM_DIVISOR_256  3
#define MM_DIVISOR_512  4
#define MM_DIVISOR_1024 5

#define MM_DIVISOR MM_DIVISOR_512


extern volatile uint8_t micromag_cur_axe;

#define MmSelect() SetBit(MM_SS_IOCLR,MM_SS_PIN)
#define MmUnselect() SetBit(MM_SS_IOSET,MM_SS_PIN)

#define MmReset() SetBit(MM_RESET_IOCLR,MM_RESET_PIN)
#define MmSet() SetBit(MM_RESET_IOSET,MM_RESET_PIN)

#define MmOnSpiIt() {							\
    switch (micromag_status) {						\
    case MM_SENDING_REQ:						\
      {									\
	/* read dummy control byte reply */				\
	uint8_t foo __attribute__ ((unused)) = SSPDR;			\
	micromag_status = MM_WAITING_EOC;				\
	MmUnselect();							\
	SSP_ClearRti();							\
	SSP_DisableRti();						\
	SSP_Disable();							\
      }									\
      break;								\
    case MM_READING_RES:						\
      {									\
	int16_t new_val;						\
	new_val = SSPDR << 8;						\
	new_val += SSPDR;						\
	if (abs(new_val) < 2000)					\
	  micromag_values[micromag_cur_axe] = new_val;			\
	MmUnselect();							\
	SSP_ClearRti();							\
	SSP_DisableRti();						\
	SSP_Disable();							\
	micromag_cur_axe++;						\
	if (micromag_cur_axe > 2) {					\
	  micromag_cur_axe = 0;						\
	  micromag_status = MM_DATA_AVAILABLE;				\
	}								\
	else								\
	  micromag_status = MM_IDLE;					\
      }									\
      break;								\
    }									\
  }


#define MmSendReq() {							\
    MmSelect();								\
    micromag_status = MM_SENDING_REQ;					\
    MmSet();								\
    SSP_ClearRti();							\
    SSP_EnableRti();							\
    MmReset();								\
    uint8_t control_byte = (micromag_cur_axe+1) << 0 | MM_DIVISOR_1024 << 4; \
    SSP_Send(control_byte);						\
    SSP_Enable();							\
  }

#define MmReadRes() {							\
    micromag_status = MM_READING_RES;					\
    MmSelect();								\
    /* trigger 2 bytes read */						\
    SSP_Send(0);							\
    SSP_Send(0);							\
    SSP_Enable();							\
    SSP_ClearRti();							\
    SSP_EnableRti();							\
  }



#endif /* MICROMAG_HW_H */
