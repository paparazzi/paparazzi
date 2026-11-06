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
#include "modules/actuators/actuators_driver.h"

/*
 * Defines SetActuatorsFromCommandsNoCommit() and driver initialization
 * Defines ACTUATORS_NB to 0 if no servo
 * Include servos drivers
 */
#include "generated/airframe.h"

extern void actuators_init(void);
extern void actuators_periodic(void);

/** Submit every registered driver once per output cycle. */
extern void actuators_commit(void);

extern struct actuator_driver actuator_drivers[ACTUATORS_DRIVERS_NB];

// Actuator feedback structure for ABI Message
struct act_feedback_t {
  uint8_t idx;                      ///< General index of the actuators (generated in airframe.h)
  struct act_feedback_set_t {
    bool rpm: 1;                    ///< RPM is set
    bool position: 1;               ///< Position is set
    bool force: 1;                  ///< Force is set
    bool speed: 1;                  ///< Speed is set
    bool power: 1;                  ///< Power rating is set
  } set;                            ///< Bitset registering what is set as feedback

  int32_t rpm;                      ///< RPM
  float position;                   ///< In radians
  float force;                      ///< In Newton or Newton metre
  float speed;                      ///< In meter per second or radian per second
  uint8_t power_pct;                ///< Power rating in percent
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

#if ACTUATORS_NB

extern uint32_t actuators_delay_time;
extern bool   actuators_delay_done;

// Actuator feedback structure for ABI Message
struct actuator_t {
  pprz_t pprz_val;            ///< Last PPRZ request; unchanged by driver-unit writes
  int16_t driver_val;         ///< Actuator value in driver units (scaling from servo in airframe.h)
  struct act_config_t config; ///< Actuator config
};

/** Actuators array.
 * */
extern struct actuator_t actuators[ACTUATORS_NB];

/** Scale, bound and submit a PPRZ value to the actuator driver immediately.
 * Buffered drivers transmit at the next commit; other drivers may transmit
 * from their setter. This function does not commit any drivers.
 * Invalid indices are ignored.
 * @param pprz_idx general actuators index
 * @param pprz_value actuator's value in pprz unit
 */
extern void actuator_set(uint8_t pprz_idx, pprz_t pprz_value);

/** Store and submit a value in driver units without scaling or bounding it.
 * The last PPRZ request is retained; no inverse conversion is performed.
 * Like actuator_set(), this calls the driver setter without committing.
 * Invalid indices are ignored.
 * @param pprz_idx general actuators index
 * @param driver_value actuator's value in driver units
 */
extern void actuator_set_driver(uint8_t pprz_idx, int16_t driver_value);

/** Set an actuator by its name from the airframe's servos section.
 * @param _n actuators name as given in airframe file, servos section
 * @param _v new actuator's value
 */
#define ActuatorSet(_n, _v) actuator_set(SERVO_ ## _n ## _IDX, _v)

/** Get current actuator value in driver units, or zero for an invalid index.
 */
extern int16_t actuator_get(uint8_t pprz_idx);

/** Get the last bounded PPRZ request, or zero for an invalid index.
 * Driver-unit writes do not change the last PPRZ request.
 */
extern pprz_t actuator_get_pprz(uint8_t pprz_idx);

/** Get global index from driver index, or ACTUATORS_NB if not found.
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
