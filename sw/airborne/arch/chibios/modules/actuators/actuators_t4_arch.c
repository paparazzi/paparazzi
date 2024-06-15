/*
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
/**
 * @file arch/chibios/modules/actuators/actuators_t4_arch.c
 * Interface from actuators to ChibiOS T4 driver
 */
#include "modules/actuators/actuators_t4_arch.h"
#include "modules/actuators/actuators_myt4.h"
//#include "mcu_periph/gpio.h"

/* Default timer base frequency is 1MHz */
#ifndef PWM_FREQUENCY
#define PWM_FREQUENCY 1000000
#endif

/* Default servo update rate in Hz */
#ifndef SERVO_HZ
#define SERVO_HZ 40
#endif

/**
 * Print the configuration variables from the header
 */
PRINT_CONFIG_VAR(ACTUATORS_T4_NB)

int32_t actuators_t4_values[ACTUATORS_T4_NB];

void actuators_t4_arch_init(void) {

}

void actuators_t4_commit(void) {

}
