/*
 * Copyright (C) 2023 Ewoud Smeur
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

#include "state.h"

float g = -9.81f;

struct FloatVect2 crash_pos;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_ballistic_touchdown(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_BALLISTIC_TOUCHDOWN(trans, dev, AC_ID,
                                &crash_pos.x,
                                &crash_pos.y);
}
#endif

void ballistic_touchdown_init(void) {
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_BALLISTIC_TOUCHDOWN, send_ballistic_touchdown);
#endif
}

/**
 * Function that predicts the ballistic crash location
 *
 * Uses ENU coordinates (vertical up!)
 **/
void ballistic_touchdown_run(void) {

  struct EnuCoor_f * v = stateGetSpeedEnu_f();
  float vz = v->z;

  struct FloatVect2 vh;
  VECT2_ASSIGN(vh, v->x, v->y);

  float h = fabsf(stateGetPositionEnu_f()->z); // Should be height above ground... CHECK!

  // With h always larger than 0, the sqrt can never give nan
  float time_fall = (-vz - sqrtf(vz*vz -2.f*h*g))/g;

  struct FloatVect2 crash_offset;

  VECT2_SMUL(crash_offset, vh, time_fall);

  struct FloatVect2 pos;
  pos.x = stateGetPositionEnu_f()->x;
  pos.y = stateGetPositionEnu_f()->y;

  VECT2_SUM(crash_pos, pos, crash_offset);

  // TODO: remove, just for testing!
  struct Int32Vect3 pos_i;
  pos_i.x = POS_BFP_OF_REAL(crash_pos.x);
  pos_i.y = POS_BFP_OF_REAL(crash_pos.y);
  pos_i.z = 0;
  uint8_t wp_id = 6;
  RunOnceEvery(100, DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice,
                                &wp_id, &pos_i.x, &pos_i.y, &pos_i.z));
}
