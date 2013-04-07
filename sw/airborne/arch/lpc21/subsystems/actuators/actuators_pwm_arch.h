/*
 * Copyright (C) 2010 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef ACTUATORS_PWM_ARCH_H
#define ACTUATORS_PWM_ARCH_H

#include "LPC21xx.h"
#include "mcu_periph/sys_time.h"


/*
   wiring on classix PWM connector
connector   LPC   shared         port
PWM1        PWM5  AD1_6  CAP1_3  P0.21
PWM2        PWM3  RXD0   EINT0   P0.1
PWM3        PWM1  TXD0           P0.0
PWM4        PWM6  RXD1   EINT3   P0.9
PWM5        PWM4  TXD1   AD1_1   P0.8
PWM6        PWM2  SSEL0  EINT2   P0.7

*/

#define SERVOS_TICS_OF_USEC(s) cpu_ticks_of_usec(s)

/* default definition of servo (from classix order) */
#ifndef SERVO_REG_0
#define SERVO_REG_0 PWMMR5
#endif
#ifndef SERVO_REG_1
#define SERVO_REG_1 PWMMR3
#endif
#ifndef SERVO_REG_2
#define SERVO_REG_2 PWMMR1
#endif
#ifndef SERVO_REG_3
#define SERVO_REG_3 PWMMR6
#endif
#ifndef SERVO_REG_4
#define SERVO_REG_4 PWMMR4
#endif
#ifndef SERVO_REG_5
#define SERVO_REG_5 PWMMR2
#endif

#define COMMAND_(i) SERVO_REG_ ## i
/** Actuator set macro */
#define ActuatorPwmSet(_i, _v) { COMMAND_(_i) = SERVOS_TICS_OF_USEC(_v); }

#define PWM_PINSEL_MASK_VAL 3

#define PWM_SERVO_1_PINSEL PINSEL0
#define PWM_SERVO_1_PINSEL_VAL 2
#define PWM_SERVO_1_PINSEL_BIT 0
#define PWM_SERVO_1_PINSEL_MASK ~(PWM_PINSEL_MASK_VAL << PWM_SERVO_1_PINSEL_BIT)
#define PWM_SERVO_1_ENA PWMPCR_ENA1
#define PWM_SERVO_1_LATCH PWMLER_LATCH1

#define PWM_SERVO_2_PINSEL PINSEL0
#define PWM_SERVO_2_PINSEL_VAL 2
#define PWM_SERVO_2_PINSEL_BIT 14
#define PWM_SERVO_2_PINSEL_MASK ~(PWM_PINSEL_MASK_VAL << PWM_SERVO_2_PINSEL_BIT)
#define PWM_SERVO_2_ENA PWMPCR_ENA2
#define PWM_SERVO_2_LATCH PWMLER_LATCH2

#define PWM_SERVO_3_PINSEL PINSEL0
#define PWM_SERVO_3_PINSEL_VAL 2
#define PWM_SERVO_3_PINSEL_BIT 2
#define PWM_SERVO_3_PINSEL_MASK ~(PWM_PINSEL_MASK_VAL << PWM_SERVO_3_PINSEL_BIT)
#define PWM_SERVO_3_ENA PWMPCR_ENA3
#define PWM_SERVO_3_LATCH PWMLER_LATCH3

#define PWM_SERVO_4_PINSEL PINSEL0
#define PWM_SERVO_4_PINSEL_VAL 2
#define PWM_SERVO_4_PINSEL_BIT 16
#define PWM_SERVO_4_PINSEL_MASK ~(PWM_PINSEL_MASK_VAL << PWM_SERVO_4_PINSEL_BIT)
#define PWM_SERVO_4_ENA PWMPCR_ENA4
#define PWM_SERVO_4_LATCH PWMLER_LATCH4

#define PWM_SERVO_5_PINSEL PINSEL1
#define PWM_SERVO_5_PINSEL_VAL 1
#define PWM_SERVO_5_PINSEL_BIT 10
#define PWM_SERVO_5_PINSEL_MASK ~(PWM_PINSEL_MASK_VAL << PWM_SERVO_5_PINSEL_BIT)
#define PWM_SERVO_5_ENA PWMPCR_ENA5
#define PWM_SERVO_5_LATCH PWMLER_LATCH5

#define PWM_SERVO_6_PINSEL PINSEL0
#define PWM_SERVO_6_PINSEL_VAL 2
#define PWM_SERVO_6_PINSEL_BIT 18
#define PWM_SERVO_6_PINSEL_MASK ~(PWM_PINSEL_MASK_VAL << PWM_SERVO_6_PINSEL_BIT)
#define PWM_SERVO_6_ENA PWMPCR_ENA6
#define PWM_SERVO_6_LATCH PWMLER_LATCH6

extern const uint8_t pwm_latch_value;

/** Actuator commit macro */
#define ActuatorsPwmCommit() { \
    PWMLER = pwm_latch_value;   \
  }

#endif /* ACTUATORS_PWM_ARCH_H */
