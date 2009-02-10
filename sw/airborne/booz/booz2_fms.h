/*
 * $id$
 * This is the "external interface" to the autopilot. It allows an external device to
 * fetch the vehicle state and input commands at different levels. We should support
 * different hardware peripherals like i2c, spi or uart.
 * For now we only have an implementation using datalink messages. 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef BOOZ2_FMS_H
#define BOOZ2_FMS_H

#include "std.h"
#include "booz_geometry_int.h"
#include "booz2_autopilot.h"
#include "booz2_guidance_h.h"
#include "booz2_guidance_v.h"


struct Booz_fms_imu_info {
  struct Pprz_int16_vect3 gyro;
  struct Pprz_int16_vect3 accel;
  struct Pprz_int16_vect3 mag;
};

struct Booz_fms_gps_info {
  struct Pprz_int32_vect3 pos;
  struct Pprz_int16_vect3 speed;
  int32_t pacc;
  uint8_t num_sv;
  uint8_t fix;
};

struct Booz_fms_ahrs_info {
  struct Pprz_int16_euler euler;
  struct Pprz_int16_rate  rate;
};

struct Booz_fms_info {
  struct Booz_fms_imu_info  imu;
  struct Booz_fms_gps_info  gps;
  struct Booz_fms_ahrs_info ahrs;
//  struct Booz_fms_ins_info  ins;
};

struct Booz_fms_command {
  union {
    struct booz_ivect  rate;
    struct booz_ieuler attitude;
    struct booz_ivect2 speed;
    struct booz_ivect pos; //FIXME Warning z is heading
  } h_sp;
  union {
    int32_t direct;
    int32_t climb;
    int32_t height;
  } v_sp;
  uint8_t h_mode;
  uint8_t v_mode;
};

extern bool_t  booz_fms_on;
extern bool_t  booz_fms_timeout;
extern uint8_t booz_fms_last_msg;

extern struct Booz_fms_info    booz_fms_info;
extern struct Booz_fms_command booz_fms_input;

extern void booz_fms_init(void);
extern void booz_fms_periodic(void);
extern void booz_fms_update_info(void);


#define BOOZ2_FMS_TYPE_DATALINK    0
#define BOOZ2_FMS_TYPE_TEST_SIGNAL 1

#if defined BOOZ2_FMS_TYPE
#if BOOZ2_FMS_TYPE == BOOZ2_FMS_TYPE_DATALINK
#include "booz2_fms_datalink.h"
#elif BOOZ2_FMS_TYPE == BOOZ2_FMS_TYPE_TEST_SIGNAL
#include "booz2_fms_test_signal.h"
#else
#error "booz2_fms.h: Unknown BOOZ2_FMS_TYPE"
#endif
#endif

#define BOOZ2_FMS_SET_POS_SP(_pos_sp,_psi_sp) { \
  _pos_sp.x = booz_fms_input.h_sp.pos.x; \
  _pos_sp.y = booz_fms_input.h_sp.pos.y; \
  /*_psi_sp = booz_fms_input.h_sp.pos.z;*/ \
}

#define BOOZ2_FMS_POS_INIT(_pos_sp,_psi_sp) { \
  booz_fms_input.h_sp.pos.x = _pos_sp.x; \
  booz_fms_input.h_sp.pos.y = _pos_sp.y; \
  booz_fms_input.h_sp.pos.z = _psi_sp; \
}

#endif /* BOOZ2_FMS_H */


