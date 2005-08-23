/*  $Id$
 *
 * (c) 2003 Pascal Brisset, Antoine Drouin
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

#ifndef LINK_AUTOPILOT_H
#define LINK_AUTOPILOT_H

#include <inttypes.h>

#include "std.h"
#include "radio.h"
#include "airframe.h"

/*
 * System clock in MHz.
 */
#define CLOCK		16

typedef int16_t pprz_t; // type of commands

/* !!!!!!!!!!!!!!!!!!! Value used in gen_airframe.ml !!!!!!!!!!!!!!!!! */
#define MAX_PPRZ (600 * CLOCK)
#define MIN_PPRZ -MAX_PPRZ
#define TRIM_PPRZ(pprz) (pprz <  MIN_PPRZ ? MIN_PPRZ :  \
                         (pprz >  MAX_PPRZ ? MAX_PPRZ : \
                                   pprz))
#define TRIM_UPPRZ(pprz) (pprz <  0 ? 0 :  \
                          (pprz >  MAX_PPRZ ? MAX_PPRZ : \
                                    pprz))


struct inter_mcu_msg {
  pprz_t channels[RADIO_CTL_NB];  
  uint8_t ppm_cpt;
  uint8_t status;
  uint8_t nb_err;
  uint8_t vsupply; /* 1e-1 V */
#if defined SECTION_IMU_ANALOG || defined SECTION_IMU_3DMG
  int16_t euler_dot[3];
#endif
#ifdef SECTION_IMU_3DMG
  int16_t euler[3];
#endif
};

// Status bits from FBW to AUTOPILOT
#define STATUS_RADIO_OK 0
#define RADIO_REALLY_LOST 1
#define STATUS_MODE_AUTO 2
#define STATUS_MODE_FAILSAFE 3
#define AVERAGED_CHANNELS_SENT 4
#define MASK_FBW_CHANGED 0xf

// Statut bits from AUTOPILOT to FBW
#define STATUS_AUTO_OK  0

/** Two bytes for the CRC */
#define FRAME_LENGTH (sizeof(struct inter_mcu_msg)+2)

#define CRC_INIT 0xffff
#define CrcUpdate(_crc, _data) _crc_ccitt_update(_crc, _data)
#define Crc1(x) ((x)&0xff)
#define Crc2(x) ((x)>>8)

#define TRESHOLD_MANUAL_PPRZ (MIN_PPRZ / 2)

#endif // LINK_AUTOPILOT_H
