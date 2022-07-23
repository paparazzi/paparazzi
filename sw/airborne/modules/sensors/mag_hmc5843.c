/*
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
#include "mcu_periph/i2c.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"
#include <math.h>
#include "modules/core/abi.h"
#include "../../peripherals/hmc5843.h"


int32_t mag_x, mag_y, mag_z;
bool mag_valid;




void hmc5843_module_init(void)
{
  hmc5843_init();
}

void hmc5843_module_periodic(void)
{
  hmc5843_periodic();
  mag_x = hmc5843.data.value[0];
  mag_y = hmc5843.data.value[1];
  mag_z = hmc5843.data.value[2];
  uint8_t id = MAG_HMC58XX_SENDER_ID;
  RunOnceEvery(30, DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice, &id, &mag_x, &mag_y, &mag_z));
}

void hmc5843_module_event(void)
{
  hmc5843_idle_task();
}
