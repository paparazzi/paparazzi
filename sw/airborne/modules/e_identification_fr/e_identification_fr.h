/*
 * Copyright (C) Fabien Bonneval
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
/**
 * @file "modules/e_identification_fr/e_identification_fr.h"
 * @author Fabien Bonneval
 * Format and send via UART tracking data according to French law.
 */

#ifndef E_IDENTIFICATION_FR_H
#define E_IDENTIFICATION_FR_H

enum e_id_type {
  // 0 reserved for future use
  E_ID_PROTOCOL_VERSION = 1,
  E_ID_ID_FR = 2,
  E_ID_ID_ANSI_UAS = 3,
  E_ID_LAT = 4,
  E_ID_LON = 5,
  E_ID_HMSL = 6,
  E_ID_HAGL = 7,
  E_ID_LAT_TO = 8,
  E_ID_LON_TO = 9,
  E_ID_H_SPEED = 10,
  E_ID_ROUTE = 11
  // 12 to 200 are reserved for future use
};

extern void e_identification_fr_init(void);
extern void e_identification_fr_periodic(void);

#define E_Identification_Fr_Start() (e_identification_fr_e_identification_fr_periodic_status = MODULES_START)
#define E_Identification_Fr_Stop() (e_identification_fr_e_identification_fr_periodic_status = MODULES_STOP)

#endif

