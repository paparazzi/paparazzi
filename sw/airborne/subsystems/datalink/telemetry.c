/*
 * Copyright (C) 2013 Gautier Hattenberger
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

#include "subsystems/datalink/telemetry_common.h"

//struct pprz_telemetry telemetry[PERIODIC_TELEMETRY_NB];

//void periodic_telemetry_init(void) {
//  telemetry = PERIODIC_TELEMETRY_MESSAGES;
//}

bool_t register_periodic_telemetry(struct pprz_telemetry * _pt, char * _msg, telemetry_cb _cb) {
  // look for message name
  uint8_t i;
  for (i = 0; i < _pt->nb; i++) {
    if (str_equal(_pt->msgs[i].msg, _msg)) {
      // register callback if not already done
      if (_pt->msgs[i].cb == NULL) {
        _pt->msgs[i].cb = _cb;
        return TRUE;
      }
      else { return FALSE; }
    }
  }
  // message name is not in telemetry file
  return FALSE;
}

