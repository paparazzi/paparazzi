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
    bool force: 1;                  ///< Force is set
    bool speed: 1;                  ///< Speed is set
    bool power: 1;                  ///< Power rating is set
    bool current: 1;                ///< Current is set
    bool voltage: 1;                ///< Voltage is set
    bool temperature: 1;            ///< Temperature is set
    bool motor_temperature: 1;      ///< Motor temperature is set
    bool output: 1;                 ///< Output duty cycle is set
    bool errors: 1;                 ///< Errors are set
  } set;                            ///< Bitset registering what is set as feedback

  int32_t rpm;                      ///< RPM
  float position;                   ///< In radians
  float force;                      ///< In Newton or Newton metre
  float speed;                      ///< In meter per second or radian per second
  uint8_t power_pct;                ///< Power rating in percent
  float current;                    ///< Current (A)
  float voltage;                    ///< Voltage (V)
  float temperature;                ///< Temperature of the actuator or ESC (deg celsius)
  float motor_temperature;          ///< Temperature of the motor (deg celsius)
  uint8_t output_pct;               ///< Output duty cycle of the ESC to the motor (percent)
  uint32_t errors;                  ///< Error count or error flags (vendor specific)
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

/** Valid bits of the actuator telemetry (as the valid field of ESC_TELEMETRY) */
#define ACT_TELEM_RPM         (1 << 0)
#define ACT_TELEM_CURRENT     (1 << 1)
#define ACT_TELEM_VOLTAGE     (1 << 2)
#define ACT_TELEM_TEMP        (1 << 3)
#define ACT_TELEM_MOTOR_TEMP  (1 << 4)
#define ACT_TELEM_OUTPUT      (1 << 5)
#define ACT_TELEM_ERRORS      (1 << 6)
#define ACT_TELEM_POSITION    (1 << 7)
#define ACT_TELEM_SPEED       (1 << 8)
#define ACT_TELEM_FORCE       (1 << 9)

/** Latest feedback of an actuator */
struct actuator_telemetry_t {
  float time;                 ///< Time of the latest feedback (s), 0 when none
  uint16_t valid;             ///< Values received (ACT_TELEM_* bits)
  bool esc;                   ///< The actuator sends ESC telemetry (RPM, output or motor temperature)
  bool servo;                 ///< The actuator sends servo telemetry (position, speed or force), and is not an ESC
  int32_t rpm;                ///< RPM
  float current;              ///< Current (A)
  float voltage;              ///< Voltage (V)
  float temperature;          ///< ESC temperature (deg celsius)
  float motor_temperature;    ///< Motor temperature (deg celsius)
  uint8_t output_pct;         ///< Output duty cycle (percent)
  uint32_t errors;            ///< Error count or error flags (vendor specific)
  float position;             ///< Position (rad)
  float speed;                ///< Speed (rad/s or m/s)
  float force;                ///< Force or torque (N or Nm)
};

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
extern struct actuator_telemetry_t actuators_telemetry[ACTUATORS_NB]; ///< Latest feedback of each actuator

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
