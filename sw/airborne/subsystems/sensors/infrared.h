/*
 * Copyright (C) 2003-2010 The Paparazzi Team
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

#ifndef SUBSYSTEMS_SENSORS_INFRARED_H
#define SUBSYSTEMS_SENSORS_INFRARED_H

#include "std.h"
#include "generated/airframe.h"

/*
 * Sensor installation
 */

#ifndef IR_IR1_SIGN
#define IR_IR1_SIGN 1
#endif /* IR_IR1_SIGN */

#ifndef IR_IR2_SIGN
#define IR_IR2_SIGN 1
#endif /* IR_IR2_SIGN */

#ifndef IR_TOP_SIGN
#define IR_TOP_SIGN 1
#endif /* IR_TOP_SIGN */

#if defined IR_HORIZ_SENSOR_ALIGNED
/* IR1 on the lateral axis, IR2 on the longitudal axis */
#define IR_RollOfIrs(_ir1, _ir2) (_ir1)
#define IR_PitchOfIrs(_ir1, _ir2) (_ir2)
#elif IR_HORIZ_SENSOR_TILTED
/* IR1 rear-left -- front-right, IR2 rear-right -- front-left
   IR1_SIGN and IR2_SIGN give positive values when it's warm on the right side
*/
#define IR_RollOfIrs(_ir1, _ir2) (_ir1 + _ir2)
#define IR_PitchOfIrs(_ir1, _ir2) (-(_ir1) + _ir2)
#else
#ifndef SITL
#error "You have to define either HORIZ_SENSOR_ALIGNED or HORIZ_SENSOR_TILTED in the IR section"
#endif
#endif
/* Vertical sensor, TOP_SIGN gives positice values when it's warm on the bottom */
#ifndef IR_TopOfIr
#define IR_TopOfIr(_ir) (_ir)
#endif

/*
 * Default correction values
 */

#ifndef IR_LATERAL_CORRECTION
#define IR_LATERAL_CORRECTION 1.
#endif

#ifndef IR_LONGITUDINAL_CORRECTION
#define IR_LONGITUDINAL_CORRECTION 1.
#endif

#ifndef IR_VERTICAL_CORRECTION
#define IR_VERTICAL_CORRECTION 1.
#endif

#ifndef IR_CORRECTION_LEFT
#define IR_CORRECTION_LEFT 1.
#endif

#ifndef IR_CORRECTION_RIGHT
#define IR_CORRECTION_RIGHT 1.
#endif

#ifndef IR_CORRECTION_UP
#define IR_CORRECTION_UP 1.
#endif

#ifndef IR_CORRECTION_DOWN
#define IR_CORRECTION_DOWN 1.
#endif


/*
 * Default neutral values
 */
#ifndef IR_ROLL_NEUTRAL_DEFAULT
#define IR_ROLL_NEUTRAL_DEFAULT 0.0
#endif

#ifndef IR_PITCH_NEUTRAL_DEFAULT
#define IR_PITCH_NEUTRAL_DEFAULT 0.0
#endif

struct Infrared_raw {
  /* the 3 channels of the sensor
   */
  int16_t ir1;
  int16_t ir2;
  int16_t ir3;
};

/** Infrared structure */
struct Infrared {
  /* raw infrared values
   */
  struct Infrared_raw value;
  /* neutrals in radians
   */
  float roll_neutral;
  float pitch_neutral;
  float pitch_vneutral;
  /* roll, pitch, top unscaled reading
   */
  int16_t roll;
  int16_t pitch;
  int16_t top;
  /* coefficients used to compensate
     for sensors sensitivity
  */
  float lateral_correction;
  float longitudinal_correction;
  float vertical_correction;
  /* coefficients used to compensate
     for masking
  */
  float correction_left;
  float correction_right;
  float correction_up;
  float correction_down;
};

extern struct Infrared infrared;

#define UpdateIRValue(_v) {                  \
    infrared.value.ir1 = (IR_IR1_SIGN)*_v.ir1; \
    infrared.value.ir2 = (IR_IR2_SIGN)*_v.ir2; \
    infrared.value.ir3 = (IR_TOP_SIGN)*_v.ir3; \
    infrared.roll = infrared.lateral_correction * IR_RollOfIrs(infrared.value.ir1, infrared.value.ir2); \
    infrared.pitch = infrared.longitudinal_correction * IR_PitchOfIrs(infrared.value.ir1, infrared.value.ir2); \
    infrared.top = infrared.vertical_correction * IR_TopOfIr(infrared.value.ir3); \
  }

// initialization of the infrared structure
void infrared_struct_init(void);

// implementation dependent functions
void infrared_init(void);
void infrared_update(void);
void infrared_event(void);

#endif /* SUBSYSTEMS_SENSORS_INFRARED_H */
