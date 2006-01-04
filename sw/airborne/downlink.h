/*
 * Paparazzi mcu0 $Id$
 *  
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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

#ifndef DOWNLINK_H
#define DOWNLINK_H

#include "modem.h"

#ifdef SITL
#include "sitl_messages.h"
#else
#include "messages.h"
#endif

#ifdef DOWNLINK
#define Downlink(x) x
#else
#define Downlink(x) {}
#endif

/** Downlink is done with the modem for the time being
    These macro could be moved to another .h */
#define DownlinkCheckFreeSpace(_x) ModemCheckFreeSpace(_x)
#define DownlinkStartMessage(_x) ModemStartMessage(_x)
#define DownlinkEndMessage(_x) ModemEndMessage(_x)
#define DownlinkPut1ByteByAddr(_x) ModemPut1ByteByAddr(_x)
#define DownlinkPut2ByteByAddr(_x) ModemPut2ByteByAddr(_x)
#define DownlinkPut4ByteByAddr(_x) ModemPut4ByteByAddr(_x)
#define downlink_nb_ovrn modem_nb_ovrn
#define DownlinkSizeOf(_x) ModemSizeOf(_x)

#define PERIODIC_SEND_IDENT()  DOWNLINK_SEND_IDENT(&ac_ident);

#define PERIODIC_SEND_BAT() Downlink({ int16_t e = energy; DOWNLINK_SEND_BAT(&desired_gaz, &vsupply, &estimator_flight_time, &low_battery, &block_time, &stage_time, &e); })

#ifdef MCU_SPI_LINK
#define PERIODIC_SEND_DEBUG_MCU_LINK() DOWNLINK_SEND_DEBUG_MCU_LINK(&link_fbw_nb_err, &link_fbw_fbw_nb_err, &mcu1_ppm_cpt);
#else
#define PERIODIC_SEND_DEBUG_MCU_LINK() {}
#endif

#define PERIODIC_SEND_DEBUG_MODEM() DOWNLINK_SEND_DEBUG_MODEM(&modem_nb_ovrn)
#define PERIODIC_SEND_ATTITUDE() Downlink({ \
  int8_t phi = DegOfRad(estimator_phi); \
  int8_t psi = DegOfRad(estimator_psi); \
  int8_t theta = DegOfRad(estimator_theta); \
  DOWNLINK_SEND_ATTITUDE(&phi, &psi, &theta); \
})

#define PERIODIC_SEND_PPRZ_MODE() DOWNLINK_SEND_PPRZ_MODE(&pprz_mode, &vertical_mode, &lateral_mode, &horizontal_mode, &inflight_calib_mode, &mcu1_status, &ir_estim_mode);
#define PERIODIC_SEND_DESIRED() DOWNLINK_SEND_DESIRED(&desired_roll, &desired_pitch, &desired_x, &desired_y, &desired_altitude, &desired_climb);

#define PERIODIC_SEND_NAVIGATION_REF()  DOWNLINK_SEND_NAVIGATION_REF(&nav_utm_east0, &nav_utm_north0, &nav_utm_zone0);

#ifdef DATALINK
#define PERIODIC_SEND_ACINFO() { \
  struct ac_info_ *s=get_ac_info(3); \
  DOWNLINK_SEND_ACINFO(&s->east, &s->north, &s->course, &s->alt, &s->gspeed); \
}
#else
#define PERIODIC_SEND_ACINFO() {}
#endif

#ifdef RADIO_CALIB
#define PERIODIC_SEND_SETTINGS() if (inflight_calib_mode != IF_CALIB_MODE_NONE)	DOWNLINK_SEND_SETTINGS(&slider_1_val, &slider_2_val);
#else
#define PERIODIC_SEND_SETTINGS() {}
#endif

#ifdef INFRARED
#define PERIODIC_SEND_ADC() DOWNLINK_SEND_ADC(&ir_roll, &ir_pitch);
#define SEND_RAD_OF_IR() Downlink({ int16_t rad = DeciDegOfRad(estimator_rad); DOWNLINK_SEND_RAD_OF_IR(&ir_roll, &rad, &estimator_rad_of_ir);})
#define PERIODIC_SEND_CALIB_START() if (!estimator_flight_time && calib_status == WAITING_CALIB_CONTRAST) { DOWNLINK_SEND_CALIB_START(); }
#define PERIODIC_SEND_CALIB_CONTRAST() if (!estimator_flight_time && calib_status == CALIB_DONE) { DOWNLINK_SEND_CALIB_CONTRAST(&ir_contrast); }
#else
#define PERIODIC_SEND_ADC() {}
#define SEND_RAD_OF_IR() {}
#define PERIODIC_SEND_CALIB_START() {}
#define PERIODIC_SEND_CALIB_CONTRAST() {}
#endif

#define PERIODIC_SEND_CALIBRATION() DOWNLINK_SEND_CALIBRATION(&climb_sum_err, &climb_pgain, &course_pgain)

#define PERIODIC_SEND_CIRCLE() if (in_circle) { DOWNLINK_SEND_CIRCLE(&circle_x, &circle_y, &circle_radius); }

#define PERIODIC_SEND_SEGMENT() if (in_segment) { DOWNLINK_SEND_SEGMENT(&segment_x_1, &segment_y_1, &segment_x_2, &segment_y_2); }

#ifdef IMU_ANALOG
#define PERIODIC_SEND_IMU() { int16_t dummy = 42; DOWNLINK_SEND_IMU(&(from_fbw.euler_dot[0]), &(from_fbw.euler_dot[1]), &(from_fbw.euler_dot[2]), &dummy, &dummy, &dummy); }
#else
#define PERIODIC_SEND_IMU() {}
#endif

#define SEND_NAVIGATION() Downlink({ int16_t pos_x = estimator_x; int16_t pos_y = estimator_y; int16_t d_course = DeciDegOfRad(desired_course); DOWNLINK_SEND_NAVIGATION(&nav_block, &nav_stage, &pos_x, &pos_y, &d_course, &dist2_to_wp, &dist2_to_home);})

#define SEND_CAM() Downlink({ int16_t x = target_x; int16_t y = target_y; int8_t phi = DegOfRad(phi_c); int8_t theta = DegOfRad(theta_c); DOWNLINK_SEND_CAM(&phi, &theta, &x, &y);})


#endif /* DOWNLINK_H */
