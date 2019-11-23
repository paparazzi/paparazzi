/*
 * Copyright (C) 2016 Hector Garcia de Marina
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */
#include "math/pprz_algebra_float.h"
#include "subsystems/abi.h"
#include "subsystems/datalink/datalink.h" // dl_buffer
#include "autopilot.h"
#include "modules/multi/fc_rotor/fc_rotor.h"
#include "firmwares/rotorcraft/navigation.h"

void fc_rotor_init(void)
{
}

void fc_read_msg(void)
{
  struct FloatVect3 u;
  uint8_t ac_id = DL_DESIRED_SETPOINT_ac_id(dl_buffer);

  if (ac_id == AC_ID) {
    // 0: 2D control, 1: 3D control
    uint8_t flag = DL_DESIRED_SETPOINT_flag(dl_buffer);

    u.x = DL_DESIRED_SETPOINT_ux(dl_buffer);
    u.y = DL_DESIRED_SETPOINT_uy(dl_buffer);
    u.z = DL_DESIRED_SETPOINT_uz(dl_buffer);

    AbiSendMsgACCEL_SP(ACCEL_SP_FCR_ID, flag, &u);

    if (flag == 0) {
      // with 2D control, set flight altitude in integer ENU LTP frame
      nav_flight_altitude = POS_BFP_OF_REAL(u.z);
    }
  }
}

