/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
 * Copyright (C) 2022 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file modules/intermcu/intermcu_ap.h
 *  @brief Inter-MCU on the AP side
 */

#ifndef INTERMCU_AP_H
#define INTERMCU_AP_H

#include "modules/intermcu/intermcu.h"
#include "generated/airframe.h"

/** send command vector over intermcu link instead of actuators
 */
extern void intermcu_send_commands(pprz_t *command_values, uint8_t ap_mode);

/** send binding signal for spektrum receiver
 */
extern void intermcu_send_spektrum_bind(void);

/** enable/disable intermcu link
 */
extern void intermcu_set_enabled(bool value);

/** Datalink event functions
 */
extern void intermcu_parse_IMCU_FBW_STATUS(uint8_t *buf);

/* Structure for FBW status */
struct fbw_status_t {
  uint8_t rc_status;
  uint8_t frame_rate;
  uint8_t mode;
};

#endif /* INTERMCU_AP_H */

