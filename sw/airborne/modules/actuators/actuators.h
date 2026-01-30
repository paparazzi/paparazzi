/*
 * (c) 2003-2005 Pascal Brisset, Antoine Drouin
 * (c) 2012 Gautier Hattenberger
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
 */

/** @file modules/actuators/actuators.h
 *  Hardware independent API for actuators (servos, motor controllers).
 *
 */
#ifndef ACTUATORS_H
#define ACTUATORS_H

#include "paparazzi.h"
#include "std.h"

/*
 * Defines SetActuatorsFromCommands() macro
 * Defines ACTUATORS_NB to 0 if no servo
 * Include servos drivers
 */
#include "generated/airframe.h"

extern void actuators_init(void);
extern void actuators_periodic(void);

// Actuator feedback structure for ABI Message
struct act_feedback_t {
  uint8_t idx;                      ///< General index of the actuators (generated in airframe.h)
  struct act_feedback_set_t {
    bool rpm: 1;                    ///< RPM is set
    bool position: 1;               ///< Position is set
  } set;                            ///< Bitset registering what is set as feedback

  int32_t rpm;                      ///< RPM
  float position;                   ///< In radians
};

// Actuator configuration
struct act_config_t {
  uint8_t driver_no;  ///< Driver number
  uint8_t servo_idx;  ///< Driver servo index
  int32_t min;        ///< Driver min value (or motor stop position)
  int32_t max;        ///< Driver max value (or motor max position)
  int32_t neutral;    ///< Driver neutral value (or motor idle position)
  float travel_up;    ///< Precomputed travel up coef
  float travel_down;  ///< Precomputed travel down coef
};

/** Actuator set callback
 *
 * @param driver_idx actuator index in driver frame
 * @param driver_value actuator value in driver unit
 */
typedef void (*act_set_t)(uint8_t driver_idx, int16_t driver_value);

#if ACTUATORS_NB

extern uint32_t actuators_delay_time;
extern bool   actuators_delay_done;

// Actuator feedback structure for ABI Message
struct actuator_t {
  pprz_t pprz_val;            ///< Actuator value in PPRZ units
  int16_t driver_val;         ///< Actuator value in driver units (scaling from servo in airframe.h)
  struct act_config_t config; ///< Actuator config
  act_set_t set;              ///< Actuator set callback function
};

/** Actuators array.
 * */
extern struct actuator_t actuators[ACTUATORS_NB];

/** Set actuators.
 * @param pprz_idx general actuators index
 * @param pprz_value actuator's value in pprz unit
 */
extern void actuator_set(uint8_t pprz_idx, pprz_t pprz_value);

/** Backward compatibility
 * @param _n actuators name as given in airframe file, servos section
 * @param _v new actuator's value
 */
#define ActuatorSet(_n, _v) actuator_set(SERVO_ ## _n ## _IDX, _v)

/** Get current actuator value in driver unit
 */
extern int16_t actuator_get(uint8_t pprz_idx);

/** Get global index from driver index
 */
extern uint8_t actuator_get_idx(uint8_t driver_no, uint8_t servo_idx);

/** Get min actuator value from driver index
 */
extern int32_t actuator_get_min(uint8_t driver_no, uint8_t servo_idx);

/** Get max actuator value from driver index
 */
extern int32_t actuator_get_max(uint8_t driver_no, uint8_t servo_idx);

/** Get neutral actuator value from driver index
 */
extern int32_t actuator_get_neutral(uint8_t driver_no, uint8_t servo_idx);

/** Set min actuator value from driver index
 */
extern void actuator_set_min(uint8_t driver_no, uint8_t servo_idx, int32_t min);

/** Set max actuator value from driver index
 */
extern void actuator_set_max(uint8_t driver_no, uint8_t servo_idx, int32_t max);

/** Set neutral actuator value from driver index
 */
extern void actuator_set_neutral(uint8_t driver_no, uint8_t servo_idx, int32_t neutral);

#endif /* ACTUATORS_NB */


#endif /* ACTUATORS_H */
