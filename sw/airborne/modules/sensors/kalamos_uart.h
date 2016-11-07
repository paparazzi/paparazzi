/*
 * Copyright (C) C. De Wagter
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/sensors/kalamos_uart.h"
 * @author Kevin van Hecke
 * Parrot Kalamos Nvidia tk1 stereo vision uart (RS232) communication
 */

#ifndef KALAMOS_UART_H
#define KALAMOS_UART_H

#include "std.h"
#include "generated/airframe.h"
#include "pprzlink/pprz_transport.h"
#include "math/pprz_orientation_conversion.h"
#include "math/pprz_algebra_float.h"


/* Main kalamos structure */
struct kalamos_t {
  struct link_device *device;           ///< The device which is uses for communication
  struct pprz_transport transport;      ///< The transport layer (PPRZ)
  struct OrientationReps imu_to_mag;    ///< IMU to magneto translation
  bool msg_available;                 ///< If we received a message
};


//should be exactly the same as pprz.h
struct Kalamos2PPRZPackage {
    float height;
    float descend_x;
    float descend_y;
    float descend_z;
    float joe_enu_x;
    float joe_enu_y;
    float land_enu_x;
    float land_enu_y;
    float avoid_psi;
    float avoid_rate;
    float flow_x;
    float flow_y;
    float att_calib_phi;
    float att_calib_theta;
    uint8_t status;
} __attribute__((__packed__));
extern struct Kalamos2PPRZPackage k2p_package;

//should be exactly the same as pprz.h
struct PPRZ2KalamosPackage {
    float phi;
    float theta;
    float psi;
    float qi;
    float qx;
    float qy;
    float qz;
    float gpsx;
    float gpsy;
    float gpsz;
    float geo_init_gpsx;
    float geo_init_gpsy;
    float geo_init_gpsz;
    unsigned char enables;
}__attribute__((__packed__));

extern float kalamos_search_height;
extern bool kalamos_enable_landing ;
extern bool kalamos_enable_spotsearch;
extern bool  kalamos_enable_findjoe;
extern bool kalamos_enable_opticflow;
extern bool kalamos_enable_attcalib;
extern bool kalamos_enable_videorecord;
extern float kalamos_land_xy_gain;
extern float kalamos_land_z_gain;
extern struct FloatVect3 land_cmd;

extern void kalamos_init(void);
extern void kalamos_event(void);
extern void kalamos_periodic(void);

extern void enableKalamosLandingspotSearch(bool b);
extern void enableKalamosDescent(bool b);
extern void enableKalamosOpticFlow(bool b);
extern void enableKalamosFindJoe(bool b);
extern bool enableKalamosAttCalib(bool b);
extern bool enableKalamosVideoRecord(bool b);

extern bool getKalamosReady(void);

#endif

