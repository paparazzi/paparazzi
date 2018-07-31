/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file boards/disco/actuators.h
 * Actuator driver for the disco
 *
 * Disco plane is using the same ESC (I2C) than bebop for its motor
 * and Pwm_sysfs linux driver for the PWM outputs
 */

#ifndef ACTUATORS_DISCO_H_
#define ACTUATORS_DISCO_H_

#include "std.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/pwm_sysfs.h"

// full 8-bit address
#define ACTUATORS_DISCO_ADDR          0x10

// I2C Commands
#define ACTUATORS_DISCO_SET_REF_SPEED 0x02    ///< Set reference speed
#define ACTUATORS_DISCO_GET_OBS_DATA  0x20    ///< Get observation data
#define ACTUATORS_DISCO_START_PROP    0x40    ///< Start the propellers
#define ACTUATORS_DISCO_TOGGLE_GPIO   0x4D    ///< Toggle GPIO (reset, red led, green led)
#define ACTUATORS_DISCO_STOP_PROP     0x60    ///< Stop the propellers
#define ACTUATORS_DISCO_CLEAR_ERROR   0x80    ///< Clear all current erros
#define ACTUATORS_DISCO_PLAY_SOUND    0x82    ///< Play a sound (0=stop, 1=short beep, 2=boot beep, 3=Be-Bop-Ah-Lula, negative=repeat)
#define ACTUATORS_DISCO_GET_INFO      0xA0    ///< Get version information

// PWM setup
#define ACTUATORS_DISCO_PWM_NB        6       ///< Max number of PWM channels
#define ACTUATORS_DISCO_MOTOR_IDX     0       ///< Index for motor BLDC

struct ActuatorsDisco {
  struct i2c_transaction i2c_trans; ///< I2C transaction for communicating with the Disco BLDC driver
  uint16_t motor_rpm;               ///< Motor RPM setpoint
  uint16_t rpm_obs;                 ///< Measured RPM
  struct PWM_Sysfs pwm[ACTUATORS_DISCO_PWM_NB]; ///< Array of PWM outputs
  uint8_t status;                   ///< Status flag
  bool rpm_saturated;               ///< RPM saturation flag (bit 15 in obs data)
};

#define ActuatorsDiscoSet(_i, _v) actuators_disco_set(_i, _v)
#define ActuatorsDiscoCommit() actuators_disco_commit()
#define ActuatorsDiscoInit() actuators_disco_init()

extern struct ActuatorsDisco actuators_disco;
extern void actuators_disco_set(uint8_t idx, uint16_t val);
extern void actuators_disco_commit(void);
extern void actuators_disco_init(void);

#endif /* ACTUATORS_DISCO_H_ */
