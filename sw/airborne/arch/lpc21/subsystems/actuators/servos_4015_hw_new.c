/*
 * Copyright (C) 2006  Antoine Drouin
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
#include "subsystems/actuators.h"
#include "armVIC.h"

#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"


uint16_t servos_values[_4015_NB_CHANNELS];

#define PWMMR_SERV0 PWMMR5
#define PWMMR_SERV1 PWMMR2
#define PWMLER_LATCH_SERV0 PWMLER_LATCH5
#define PWMLER_LATCH_SERV1 PWMLER_LATCH2
#define PWMMCR_MRI_SERV0 PWMMCR_MR5I
#define PWMMCR_MRI_SERV1 PWMMCR_MR2I
#define PWMPCR_ENA_SERV0 PWMPCR_ENA5
#define PWMPCR_ENA_SERV1 PWMPCR_ENA2
#define PWMIR_MRI_SERV0 PWMIR_MR5I
#define PWMIR_MRI_SERV1 PWMIR_MR2I

#ifndef PWM_VIC_SLOT
#define PWM_VIC_SLOT 3
#endif

void actuators_4015_init(void)
{
  /* PWM selected as IRQ */
  VICIntSelect &= ~VIC_BIT(VIC_PWM);
  /* PWM interrupt enabled */
  VICIntEnable = VIC_BIT(VIC_PWM);
  _VIC_CNTL(PWM_VIC_SLOT) = VIC_ENABLE | VIC_PWM;
  /* address of the ISR */
  _VIC_ADDR(PWM_VIC_SLOT) = (uint32_t)PWM_ISR;
  /* set clock, data and reset pins as output */
  IO0DIR |= _BV(SERV0_CLOCK_PIN);
  IO1DIR |= _BV(SERV0_DATA_PIN) | _BV(SERV0_RESET_PIN);
  IO0DIR |= _BV(SERV1_CLOCK_PIN);
  IO1DIR |= _BV(SERV1_DATA_PIN) | _BV(SERV1_RESET_PIN);
  /* Configure the two servo clocks as PWM  */
  SERV0_CLOCK_PINSEL |= SERV0_CLOCK_PINSEL_VAL << SERV0_CLOCK_PINSEL_BIT;
  SERV1_CLOCK_PINSEL |= SERV1_CLOCK_PINSEL_VAL << SERV1_CLOCK_PINSEL_BIT;

  /* set first pulse to be very long */
  PWMMR0 = 0XFFFFFF;
  PWMMR_SERV1 = 0XFFF;
  PWMMR_SERV0 = 0X0;
  /* commit above changes            */
  PWMLER = PWMLER_LATCH0 | PWMLER_LATCH_SERV0 | PWMLER_LATCH_SERV1;
  /* enable interrupt on serv1 PWM match    */
  PWMMCR = PWMMCR_MR0R | PWMMCR_MRI_SERV1;
  /* enable PWM ouptputs            */
  PWMPCR = PWMPCR_ENA_SERV0 | PWMPCR_ENA_SERV1;

  /* Prescaler */
  PWMPR = PWM_PRESCALER - 1;

  /* enable PWM timer counter and PWM mode  */
  PWMTCR = PWMTCR_COUNTER_ENABLE | PWMTCR_PWM_ENABLE;
  /* Load failsafe values              */
  /* Set all servos at their midpoints */
  /* compulsory for unaffected servos  */
  uint8_t i;
  for (i = 0 ; i < _4015_NB_CHANNELS ; i++) {
    servos_values[i] = SERVOS_TICS_OF_USEC(1500);
  }
#ifdef SERVO_MOTOR
  servos_values[SERVO_MOTOR] = SERVOS_TICS_OF_USEC(SERVO_MOTOR_NEUTRAL);
#endif
}



#define SERVO_REFRESH_TICS SERVOS_TICS_OF_USEC(25000)


static uint8_t servos_idx = 0;
static uint32_t servos_delay;

void PWM_ISR(void)
{
  ISR_ENTRY();
  if (servos_idx == 0) {
    /* lower serv0 reset */
    IO1CLR = _BV(SERV0_RESET_PIN);
    /* raise serv0 data  */
    IO1SET = _BV(SERV0_DATA_PIN);
    /* start pulsing serv0 */
    PWMMR_SERV0 = 0XFFF;
    /* stop pulsing serv1 */
    PWMMR_SERV1 = 0X0;
    PWMMR0 = servos_values[servos_idx];
    servos_delay = SERVO_REFRESH_TICS - servos_values[servos_idx];
    PWMLER = PWMLER_LATCH0 | PWMLER_LATCH_SERV0 | PWMLER_LATCH_SERV1;
    /* enable serv0 match interrupt, disable serv1 match interrupt */
    PWMMCR = PWMMCR_MR0R | PWMMCR_MRI_SERV0;
    servos_idx++;
  } else if (servos_idx < 4) {
    /* lower serv0 data */
    IO1CLR = _BV(SERV0_DATA_PIN);
    PWMMR0 = servos_values[servos_idx];
    servos_delay -= servos_values[servos_idx];
    PWMLER = PWMLER_LATCH0;
    servos_idx++;
  } else if (servos_idx == 4) {
    /* raise serv0 reset */
    IO1SET = _BV(SERV1_RESET_PIN);
    /* lower serv1 reset */
    IO1CLR = _BV(SERV1_RESET_PIN);
    /* raise serv1 data */
    IO1SET = _BV(SERV1_DATA_PIN);
    /* stop pulsing serv0 */
    PWMMR_SERV0 = 0;
    /* start pulsing serv1 */
    PWMMR_SERV1 = 0XFFF;
    /* disable serv0 interrupt, enable serv1 match interrupt */
    PWMMCR = PWMMCR_MR0R | PWMMCR_MRI_SERV1;
    /* fill next servo value */
    PWMMR0 = servos_values[servos_idx];
    servos_delay -= servos_values[servos_idx];
    /* latch PWM values */
    PWMLER = PWMLER_LATCH0 | PWMLER_LATCH_SERV0 | PWMLER_LATCH_SERV1;
    servos_idx++;
  } else if (servos_idx < _4015_NB_CHANNELS) {
    /* clear serv1 data */
    IO1CLR = _BV(SERV1_DATA_PIN);
    PWMMR0 = servos_values[servos_idx];
    servos_delay -= servos_values[servos_idx];
    PWMLER = PWMLER_LATCH0;
    servos_idx++;
  } else {
    /* raise serv1 reset */
    IO1SET = _BV(SERV1_RESET_PIN);
    /* command the delay */
    PWMMR0 = servos_delay;
    PWMLER = PWMLER_LATCH0;
    servos_idx = 0;
  }
  /* clear the interrupt */
  PWMIR = PWMIR_MRI_SERV1;
  PWMIR = PWMIR_MRI_SERV0;
  VICVectAddr = 0x00000000;
  ISR_EXIT();
}
