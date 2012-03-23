/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2012 Gautier Hattenberger
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef MS2100_ARCH_H
#define MS2100_ARCH_H

#define Ms2100Reset() SetBit(MS2100_RESET_IOCLR,MS2100_RESET_PIN)
#define Ms2100Set()   SetBit(MS2100_RESET_IOSET,MS2100_RESET_PIN)

#if 0
#define Ms2100OnSpiInt() {                                   \
    switch (ms2100_status) {                                \
    case MS2100_SENDING_REQ:                                \
      {                                                     \
        /* read dummy control byte reply */                 \
        uint8_t foo __attribute__ ((unused)) = SSPDR;       \
        ms2100_status = MS2100_WAITING_EOC;                 \
        Ms2100Unselect();                                   \
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
        Ms2100Unselect();                                   \
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


#define Ms2100SendReq() {                               \
    Ms2100Select();                                     \
    ms2100_status = MS2100_SENDING_REQ;					\
    Ms2100Set();                                        \
    SSP_ClearRti();                                     \
    SSP_EnableRti();                                    \
    Ms2100Reset();                                      \
    uint8_t control_byte = (ms2100_cur_axe+1) << 0 |    \
      MS2100_DIVISOR << 4;                              \
    SSP_Send(control_byte);                             \
    SSP_Enable();                                       \
  }

#define Ms2100ReadRes() {						\
    ms2100_status = MS2100_READING_RES;         \
    Ms2100Select();                             \
    /* trigger 2 bytes read */                  \
    SSP_Send(0);                                \
    SSP_Send(0);                                \
    SSP_Enable();                               \
    SSP_ClearRti();                             \
    SSP_EnableRti();							\
  }

#endif

#endif /* MS2100_ARCH_H */
