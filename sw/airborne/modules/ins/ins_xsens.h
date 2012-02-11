/*
 * Paparazzi $Id: $
 *
 * Copyright (C) 2010 ENAC
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

/**
 * \brief Library for the XSENS AHRS
 */

#ifndef INS_XSENS_H
#define INS_XSENS_H

#include "std.h"

#include "ins_module.h"

extern int8_t xsens_hour;
extern int8_t xsens_min;
extern int8_t xsens_sec;
extern int32_t xsens_nanosec;
extern int16_t xsens_year;
extern int8_t xsens_month;
extern int8_t xsens_day;
extern uint8_t xsens_msg_status;
extern uint16_t xsens_time_stamp;

#define ImuEvent(_gyro_handler, _accel_handler, _mag_handler) {	\
  InsEventCheckAndHandle(handle_ins_msg()) 			\
}


#endif
