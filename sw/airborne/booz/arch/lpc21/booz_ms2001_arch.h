#ifndef BOOZ_MS2001_ARCH_H
#define BOOZ_MS2001_ARCH_H

#include <stdlib.h>  // for abs

#include "std.h"
#include "LPC21xx.h"
#include "interrupt_hw.h" 

#include "ssp_hw.h"
#include BOARD_CONFIG

#include "airframe.h"

#define MS2001_DIVISOR_128  2
#define MS2001_DIVISOR_256  3
#define MS2001_DIVISOR_512  4
#define MS2001_DIVISOR_1024 5

#define MS2001_DIVISOR MS2001_DIVISOR_1024


extern volatile uint8_t ms2001_cur_axe;

#define Ms2001Select()   SetBit(MS2001_SS_IOCLR,MS2001_SS_PIN)
#define Ms2001Unselect() SetBit(MS2001_SS_IOSET,MS2001_SS_PIN)

#define Ms2001Reset() SetBit(MS2001_RESET_IOCLR,MS2001_RESET_PIN)
#define Ms2001Set()   SetBit(MS2001_RESET_IOSET,MS2001_RESET_PIN)

#define Ms2001OnSpiIt() {						\
    switch (ms2001_status) {						\
    case MS2001_SENDING_REQ:						\
      {									\
	/* read dummy control byte reply */				\
	uint8_t foo __attribute__ ((unused)) = SSPDR;			\
	ms2001_status = MS2001_WAITING_EOC;				\
	Ms2001Unselect();						\
	SSP_ClearRti();							\
	SSP_DisableRti();						\
	SSP_Disable();							\
      }									\
      break;								\
    case MS2001_READING_RES:						\
      {									\
	int16_t new_val;						\
	new_val = SSPDR << 8;						\
	new_val += SSPDR;						\
	if (abs(new_val) < 2000)					\
	  ms2001_values[micromag_cur_axe] = new_val;			\
	Ms2001Unselect();						\
	SSP_ClearRti();							\
	SSP_DisableRti();						\
	SSP_Disable();							\
	ms2001_cur_axe++;						\
	if (ms2001_cur_axe > 2) {					\
	  ms2001_cur_axe = 0;						\
	  ms2001_status = MS2001_DATA_AVAILABLE;			\
	}								\
	else								\
	  ms2001_status = MS2001_IDLE;					\
      }									\
      break;								\
    }									\
  }


#define Ms2001SendReq() {						\
    Ms2001Select();							\
    ms2001_status = MS2001_SENDING_REQ;					\
    Ms2001Set();							\
    SSP_ClearRti();							\
    SSP_EnableRti();							\
    Ms2001Reset();							\
    uint8_t control_byte = (ms2001_cur_axe+1) << 0 |			\
                            MS2001_DIVISOR << 4;			\
    SSP_Send(control_byte);						\
    SSP_Enable();							\
  }

#define MmReadRes() {							\
    ms2001_status = MS2001_READING_RES;					\
    Ms2001Select();							\
    /* trigger 2 bytes read */						\
    SSP_Send(0);							\
    SSP_Send(0);							\
    SSP_Enable();							\
    SSP_ClearRti();							\
    SSP_EnableRti();							\
  }



#endif /* BOOZ_MS2001_ARCH_H */
