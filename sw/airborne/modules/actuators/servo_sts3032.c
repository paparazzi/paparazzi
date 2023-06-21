/*
 * Copyright (C) 2023 Flo&Fab <surname.name@enac.fr>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/actuators/servo_sts3032.c"
 * @author Flo&Fab <surname.name@enac.fr>
 * feetech sts3032 servo 
 */

#include "modules/actuators/servo_sts3032.h"
#include "peripherals/sts3032.h"
//#include "pprzlink/messages.h"
#include "modules/datalink/telemetry.h"

struct sts3032 sts;
uint16_t val = 0;
uint8_t cbuf[100];


void servo_sts3032_init(void)
{
  sts3032_init(&sts, &(STS3032_DEV), cbuf, sizeof(cbuf));

  sts.ids[0] = 1;

  //sts3032_enable_torque(&sts, 1, 0);

  // your init code here
}

void servo_sts3032_event(void)
{
  sts3032_event(&sts);
}


int inc = 10;

void servo_sts3032_test(void)
{
  // your periodic code here.
  // freq = 1.0 Hz
  sts3032_write_pos(&sts, 1, val);

  sts3032_read_pos(&sts, 1);

  val = (val + inc)%4095;
  // if(val > 4090 || val < 5) {
  //   inc = -inc;
  // }

  float data[2] = {sts.pos[0], sts.nb_failed_checksum};

  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 2, data);
}



