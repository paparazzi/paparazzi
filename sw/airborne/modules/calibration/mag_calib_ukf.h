/*
 * Copyright (C) w.vlenterie
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
 * @file "modules/calibration/mag_calib_ukf.h"
 * @author w.vlenterie
 * Calibrate magnetometer using UKF
 */

#ifndef MAG_CALIB_UKF_H
#define MAG_CALIB_UKF_H

#ifdef AHRS_FLOAT_CMPL_WRAPPER_H
#undef AHRS_FC_MAG_ID
#define AHRS_FC_MAG_ID MAG_CALIB_UKF_ID
#endif

#ifdef AHRS_INT_CMPL_EULER_WRAPPER_H
#undef  AHRS_ICE_MAG_ID
#define AHRS_ICE_MAG_ID MAG_CALIB_UKF_ID
#endif

#ifdef AHRS_FLOAT_MLKF_WRAPPER_H
#undef  AHRS_MLKF_MAG_ID
#define AHRS_MLKF_MAG_ID MAG_CALIB_UKF_ID
#endif

#ifdef AHRS_FLOAT_INVARIANT_WRAPPER_H
#undef  AHRS_FINV_MAG_ID
#define AHRS_FINV_MAG_ID MAG_CALIB_UKF_ID
#endif

#ifdef AHRS_FLOAT_DCM_WRAPPER_H
#undef  AHRS_DCM_MAG_ID
#define AHRS_DCM_MAG_ID MAG_CALIB_UKF_ID
#endif

#ifdef AHRS_INT_CMPL_QUAT_WRAPPER_H
#undef  AHRS_ICQ_MAG_ID
#define AHRS_ICQ_MAG_ID MAG_CALIB_UKF_ID
#endif

#ifdef INS_FLOAT_INVARIANT_WRAPPER_H
#undef  INS_FINV_MAG_ID
#define INS_FINV_MAG_ID MAG_CALIB_UKF_ID
#endif

extern bool settings_reset_state;

void mag_calib_ukf_init(void);
void mag_calib_hotstart_write(void);
void mag_calib_hotstart_read(void);

#endif
