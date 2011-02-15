#ifndef MS2100_ARCH_H
#define MS2100_ARCH_H

#include <stdlib.h>  // for abs

#include "std.h"
#include "LPC21xx.h"
#include "interrupt_hw.h"

#include "ssp_hw.h"
#include BOARD_CONFIG

#include "generated/airframe.h"



extern volatile uint8_t ms2100_cur_axe;

#define Ms2001Select()   SetBit(MS2100_SS_IOCLR,MS2100_SS_PIN)
#define Ms2001Unselect() SetBit(MS2100_SS_IOSET,MS2100_SS_PIN)

#define Ms2001Reset() SetBit(MS2100_RESET_IOCLR,MS2100_RESET_PIN)
#define Ms2001Set()   SetBit(MS2100_RESET_IOSET,MS2100_RESET_PIN)

#define Ms2001OnSpiInt() {                                   \
    switch (ms2100_status) {                                \
    case MS2100_SENDING_REQ:                                \
      {                                                     \
        /* read dummy control byte reply */                 \
        uint8_t foo __attribute__ ((unused)) = SSPDR;       \
        ms2100_status = MS2100_WAITING_EOC;                 \
        Ms2001Unselect();                                   \
        SSP_ClearRti();                                     \
        SSP_DisableRti();                                   \
        SSP_Disable();                                      \
      }                                                     \
      break;                                                \
    case MS2100_READING_RES:                                \
      {                                                     \
        int16_t new_val;                                    \
        new_val = SSPDR << 8;                               \
        new_val += SSPDR;                                   \
        if (abs(new_val) < 2000)                            \
          ms2100_values[ms2100_cur_axe] = new_val;			\
        Ms2001Unselect();                                   \
        SSP_ClearRti();                                     \
        SSP_DisableRti();                                   \
        SSP_Disable();                                      \
        ms2100_cur_axe++;                                   \
        if (ms2100_cur_axe > 2) {                           \
          ms2100_cur_axe = 0;                               \
          ms2100_status = MS2100_DATA_AVAILABLE;			\
        }                                                   \
        else                                                \
          ms2100_status = MS2100_IDLE;                      \
      }                                                     \
      break;                                                \
    }                                                       \
  }


#define Ms2001SendReq() {                               \
    Ms2001Select();                                     \
    ms2100_status = MS2100_SENDING_REQ;					\
    Ms2001Set();                                        \
    SSP_ClearRti();                                     \
    SSP_EnableRti();                                    \
    Ms2001Reset();                                      \
    uint8_t control_byte = (ms2100_cur_axe+1) << 0 |    \
      MS2100_DIVISOR << 4;                              \
    SSP_Send(control_byte);                             \
    SSP_Enable();                                       \
  }

#define Ms2001ReadRes() {						\
    ms2100_status = MS2100_READING_RES;         \
    Ms2001Select();                             \
    /* trigger 2 bytes read */                  \
    SSP_Send(0);                                \
    SSP_Send(0);                                \
    SSP_Enable();                               \
    SSP_ClearRti();                             \
    SSP_EnableRti();							\
  }



#endif /* MS2100_ARCH_H */
