/*
 * $Id$
 *
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

#ifndef BOOZ_FMS_H
#define BOOZ_FMS_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "booz2_autopilot.h"
#include "booz_guidance.h"

struct Booz_fms_imu_info {
  struct Int16Vect3 gyro;
  struct Int16Vect3 accel;
  struct Int16Vect3 mag;
};

struct Booz_fms_gps_info {
  struct Int32Vect3 pos;
  struct Int16Vect3 speed;
  int32_t pacc;
  uint8_t num_sv;
  uint8_t fix;
};

struct Booz_fms_ahrs_info {
  struct Int16Eulers euler;
  struct Int16Eulers  rate;
};

struct Booz_fms_info {
  struct Booz_fms_imu_info  imu;
  struct Booz_fms_gps_info  gps;
  struct Booz_fms_ahrs_info ahrs;
  //  struct Booz_fms_ins_info  ins;
};

struct Booz_fms_command {
  union {
    struct Int32Vect3  rate;
    struct Int32Eulers attitude;
    struct Int32Vect3 speed; //FIXME Warning z is heading rate
    struct Int32Vect3 pos; //FIXME Warning z is heading
  } h_sp;
  union {
    int32_t direct;
    int32_t climb;
    int32_t height;
  } v_sp;
  uint8_t h_mode;
  uint8_t v_mode;
};

struct BoozFms {
  bool_t enabled;
  bool_t timeouted;
  uint8_t last_msg;
  struct Booz_fms_info info;
  struct Booz_fms_command input;
};

extern struct BoozFms fms;

extern void booz_fms_init(void);
extern void booz_fms_set_enabled(bool_t enabled);
extern void booz_fms_periodic(void);
extern void booz_fms_update_info(void);

/* must be implemented by specific module */
extern void booz_fms_impl_init(void);
extern void booz_fms_impl_periodic(void);
extern void booz_fms_impl_set_enabled(bool_t enabled);


#define BOOZ_FMS_TYPE_DATALINK    0
#define BOOZ_FMS_TYPE_TEST_SIGNAL 1

#if defined BOOZ_FMS_TYPE
#if BOOZ_FMS_TYPE == BOOZ_FMS_TYPE_DATALINK
#include "fms/booz_fms_datalink.h"
#elif BOOZ_FMS_TYPE == BOOZ_FMS_TYPE_TEST_SIGNAL
#include "fms/booz_fms_test_signal.h"
#else
#error "booz_fms.h: Unknown BOOZ2_FMS_TYPE"
#endif
#else /* no FMS */
#define  booz_fms_init() {}
#define  booz_fms_periodic() {}
#endif

#define BOOZ2_FMS_SET_POS_SP(_pos_sp,_psi_sp) { \
    _pos_sp.x = fms.input.h_sp.pos.x;		\
    _pos_sp.y = fms.input.h_sp.pos.y;		\
    /*_psi_sp = fms.input.h_sp.pos.z;*/		\
}

#define BOOZ2_FMS_POS_INIT(_pos_sp,_psi_sp) {	\
    fms.input.h_sp.pos.x = _pos_sp.x;		\
    fms.input.h_sp.pos.y = _pos_sp.y;		\
    fms.input.h_sp.pos.z = _psi_sp;		\
  }

#define booz_fms_SetEnabled(_val) {		\
    fms.enabled = _val;				\
    booz_fms_set_enabled(_val);			\
  }

#endif /* BOOZ_FMS_H */


