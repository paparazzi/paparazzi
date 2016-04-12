/*
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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

/** \file ins_module.h
 *  \brief Device independent INS code
 *
*/


#ifndef INS_MODULE_H
#define INS_MODULE_H

#include "std.h"
#include "subsystems/ins.h"

#ifndef INS_FORMAT
#define INS_FORMAT float
#endif

extern INS_FORMAT ins_x;
extern INS_FORMAT ins_y;
extern INS_FORMAT ins_z;

extern INS_FORMAT ins_vx;
extern INS_FORMAT ins_vy;
extern INS_FORMAT ins_vz;

extern INS_FORMAT ins_phi;
extern INS_FORMAT ins_theta;
extern INS_FORMAT ins_psi;

extern INS_FORMAT ins_p;
extern INS_FORMAT ins_q;
extern INS_FORMAT ins_r;

extern INS_FORMAT ins_ax;
extern INS_FORMAT ins_ay;
extern INS_FORMAT ins_az;

extern INS_FORMAT ins_mx;
extern INS_FORMAT ins_my;
extern INS_FORMAT ins_mz;

#if USE_INS_MODULE
extern INS_FORMAT ins_roll_neutral;
extern INS_FORMAT ins_pitch_neutral;
#endif

extern volatile uint8_t ins_msg_received;
extern volatile uint8_t new_ins_attitude;

void handle_ins_msg(void);
void parse_ins_msg(void);
void parse_ins_buffer(uint8_t);

#include "pprzlink/pprzlink_device.h"

#define InsLinkDevice (&((INS_LINK).device))

#ifndef SITL
#include "mcu_periph/uart.h"
#include "mcu_periph/spi.h"

#define InsSend1(c) InsLinkDevice->put_byte(InsLinkDevice->periph, 0, c)
#define InsUartSend1(c) InsSend1(c)
#define InsSend(_dat,_len) { for (uint8_t i = 0; i< (_len); i++) InsSend1(_dat[i]); };
#define InsUartSetBaudrate(_b) uart_periph_set_baudrate(INS_LINK, _b)

#endif /** !SITL */

static inline void ins_event_check_and_handle(void (* handler)(void))
{
  struct link_device *dev = InsLinkDevice;
  if (dev->char_available(dev->periph)) {
    while (dev->char_available(dev->periph) && !ins_msg_received) {
      parse_ins_buffer(dev->get_byte(dev->periph));
    }
  }
  if (ins_msg_received) {
    parse_ins_msg();
    handler();
    ins_msg_received = false;
  }
}

#endif /* INS_MODULE_H */
