/*
 * Copyright (C) 2009-2010 The Paparazzi Team
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
 *
 */

#ifndef RADIO_CONTROL_SPEKTRUM_ARCH_H
#define RADIO_CONTROL_SPEKTRUM_ARCH_H

#include "std.h"
#include "mcu_periph/uart.h"

#include RADIO_CONTROL_SPEKTRUM_MODEL_H

#define RC_SPK_SYNC_1 0x03

#define RC_SPK_STA_UNINIT     0
#define RC_SPK_STA_GOT_SYNC_1 1
#define RC_SPK_STA_GOT_SYNC_2 2

extern bool   rc_spk_parser_status;
extern uint8_t  rc_spk_parser_idx;
extern uint8_t  rc_spk_parser_buf[RADIO_CONTROL_NB_CHANNEL * 2];

#define MAX_SPK 344


extern const int16_t rc_spk_throw[RADIO_CONTROL_NB_CHANNEL];

#define __RcLink(dev, _x) dev##_x
#define _RcLink(dev, _x)  __RcLink(dev, _x)
#define RcLink(_x) _RcLink(RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT, _x)

#define RcLinkChAvailable() RcLink(ChAvailable())
#define RcLinkGetCh() RcLink(Getch())

#define RadioControlEventImpl(_received_frame_handler) {                \
    while (RcLinkChAvailable()) {                                       \
      int8_t c = RcLinkGetCh();                                         \
      switch (rc_spk_parser_status) {                                   \
        case RC_SPK_STA_UNINIT:                                           \
          if (c==RC_SPK_SYNC_1)                                           \
            rc_spk_parser_status = RC_SPK_STA_GOT_SYNC_1;                 \
          break;                                                          \
        case RC_SPK_STA_GOT_SYNC_1:                                       \
          if (c==RC_SPK_SYNC_2) {                                         \
            rc_spk_parser_status = RC_SPK_STA_GOT_SYNC_2;                 \
            rc_spk_parser_idx = 0;                                        \
          }                                                               \
          else                                                            \
            rc_spk_parser_status = RC_SPK_STA_UNINIT;                     \
          break;                                                          \
        case RC_SPK_STA_GOT_SYNC_2:                                       \
          rc_spk_parser_buf[rc_spk_parser_idx] = c;                       \
          rc_spk_parser_idx++;                                            \
          if (rc_spk_parser_idx >= 2*RADIO_CONTROL_NB_CHANNEL) {          \
            rc_spk_parser_status = RC_SPK_STA_UNINIT;                     \
            radio_control.frame_cpt++;                                    \
            radio_control.time_since_last_frame = 0;                      \
            radio_control.status = RC_OK;                                 \
            uint8_t i;                                                    \
            for (i=0;i<RADIO_CONTROL_NB_CHANNEL;i++) {                    \
              const int16_t tmp = (rc_spk_parser_buf[2*i]<<8) +           \
                                  rc_spk_parser_buf[2*i+1];                                 \
              /*const int16_t chan = (tmp&0xFC00) >> 10;*/                \
              const int16_t val  = (tmp&0x03FF) - 512;                    \
              radio_control.values[i] = val;                              \
              radio_control.values[i] *= rc_spk_throw[i];                 \
              if (i==RADIO_CONTROL_THROTTLE) {                            \
                radio_control.values[i] += MAX_PPRZ;                      \
                radio_control.values[i] /= 2;                             \
              }                                                           \
            }                                                             \
            _received_frame_handler();                                    \
          }                                                               \
          break;                                                          \
        default:                                                          \
          rc_spk_parser_status = RC_SPK_STA_UNINIT;                       \
      }                                                                 \
    }                                                                   \
  }


#endif /* RADIO_CONTROL_SPEKTRUM_ARCH_H */
