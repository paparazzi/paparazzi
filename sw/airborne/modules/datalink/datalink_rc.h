/*
 * Copyright (C) 2010-2014 The Paparazzi Team
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
 */

/**
 * @file modules/datalink/datalink_rc.h
 *
 * Radio control input via datalink for split AP/FBW aputopilots
 * such as PX4
 */

#ifndef DATALINK_RC_H
#define DATALINK_RC_H

#include "std.h"

#define RC_DL_NB_CHANNEL 6
extern volatile bool rc_dl_frame_available;

void datalink_rc_init(void);
void dl_parse_datalink_rc_3ch(void);
void dl_parse_datalink_rc_4ch(void);
void dl_parse_datalink_rc_5ch(void);
void intermcu_send_datalink_rc(void);


#endif /* DATALINK_RC_H */
