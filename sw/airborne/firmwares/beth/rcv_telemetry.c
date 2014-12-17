/*
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
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
/** \file rcv_telemetry.c
 *  \brief Handling of messages coming from other A/Cs
 *
 */
#define DATALINK_C

#define MODULES_DATALINK_C

#include <inttypes.h>
#include <string.h>
#include "subsystems/datalink/datalink.h"

#include "generated/modules.h"

#include <stdio.h>
#include "subsystems/navigation/common_nav.h"
#include "generated/settings.h"


#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"



#define MOfCm(_x) (((float)(_x))/100.)

#define SenderIdOfMsg(x) (x[0])
#define IdOfMsg(x) (x[1])

float phi;
float psi;
float theta;
uint16_t throttle;
uint16_t voltage;
uint16_t amps;
uint16_t energy;
uint16_t adc1;
uint16_t adc2;

/*
 6 ATTITUDE
 8 GPS
10 NAVIGATION
12 BAT
16 DESIRED
35
36
37 ENERGY
42 ESTIMATOR
*/
void dl_parse_msg(void)
{
  datalink_time = 0;
  uint8_t msg_id = IdOfMsg(dl_buffer);
  printf("Tiny rx id: %d\n", msg_id);

  if (msg_id == DL_ATTITUDE) {
    phi = DL_ATTITUDE_phi(dl_buffer);
    psi = DL_ATTITUDE_psi(dl_buffer);
    theta = DL_ATTITUDE_theta(dl_buffer);
    printf("Attitude: %f %f %f\n", phi, psi, theta);
  }
  if (msg_id == DL_BAT) {
    throttle = DL_BAT_throttle(dl_buffer);
    voltage = DL_BAT_voltage(dl_buffer);
    amps = DL_BAT_amps(dl_buffer);
    energy = DL_BAT_energy(dl_buffer);
    printf("BAT: %d %d %d %d\n", throttle, voltage, amps, energy);
  }
  if (msg_id == DL_ADC_GENERIC) {
    adc1 = DL_ADC_GENERIC_val1(dl_buffer);
    adc2 = DL_ADC_GENERIC_val2(dl_buffer);
    printf("ADC: %d %d\n", adc1, adc2);
  }

  else {
    //printf("Tiny msg_id %d unknown\n",msg_id);
  }

}
