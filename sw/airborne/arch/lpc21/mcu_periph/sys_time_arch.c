/*
 *
 * Copyright (C) 2009-2011 The Paparazzi Team
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

/**
 * @file arch/lpc21/mcu_periph/sys_time_arch.c
 * @ingroup lpc21_arch
 *
 * LPC21xx timing functions.
 */

#include "mcu_periph/sys_time.h"

#include "armVIC.h"

#ifdef SYS_TIME_LED
#include "led.h"
#endif

#define SYS_TICK_IT TIR_MR0I

#if defined ACTUATORS && ( defined SERVOS_4017 || defined SERVOS_4015_MAT || defined SERVOS_PPM_MAT)
#ifdef SERVOS_4015_MAT
#include "subsystems/actuators/servos_4015_MAT_hw.h"
#endif
#ifdef SERVOS_4017
#include "subsystems/actuators/servos_4017_hw.h"
#endif
#ifdef SERVOS_PPM_MAT
#include "subsystems/actuators/servos_ppm_hw.h"
#endif
#else
#define ACTUATORS_IT 0x00
#endif /* ACTUATORS */

#if defined RADIO_CONTROL && defined RADIO_CONTROL_TYPE_PPM
#include "subsystems/radio_control.h"
#else
#define PPM_IT 0x00
#endif

#ifdef MB_SCALE
#include "mb_scale.h"
#else
#define MB_SCALE_IT 0x00
#endif

#ifdef MB_TACHO
#include "mb_tacho.h"
#else
#define MB_TACHO_IT 0x00
#endif

#ifdef USE_PWM_INPUT
#include "mcu_periph/pwm_input.h"
#endif
#ifndef USE_PWM_INPUT1
#define PWM_INPUT_IT1 0x00
#endif
#ifndef USE_PWM_INPUT2
#define PWM_INPUT_IT2 0x00
#endif

#ifdef USE_AMI601
#include "peripherals/ami601.h"
#else
#define AMI601_IT 0x00
#endif

#ifdef TRIGGER_EXT
#include "core/trigger_ext_hw.h"
#else
#define TRIGGER_IT 0x00
#endif

#define TIMER0_IT_MASK (SYS_TICK_IT          |  \
                        ACTUATORS_IT         |  \
                        PPM_IT               |  \
                        TRIGGER_IT           |  \
                        MB_SCALE_IT          |  \
                        MB_TACHO_IT          |  \
                        PWM_INPUT_IT1        |  \
                        PWM_INPUT_IT2        |  \
                        AMI601_IT)


void sys_time_arch_init(void)
{
  sys_time.cpu_ticks_per_sec = PCLK / T0_PCLK_DIV;
  /* cpu ticks per desired sys_time timer step */
  sys_time.resolution_cpu_ticks = (uint32_t)(sys_time.resolution * sys_time.cpu_ticks_per_sec + 0.5);

  /* setup Timer 0 to count forever  */
  /* reset & disable timer 0         */
  T0TCR = TCR_RESET;
  /* set the prescale divider        */
  T0PR = T0_PCLK_DIV - 1;
  /* enable interrupt on match0  for sys_ticks */
  T0MCR = TMCR_MR0_I;
  /* disable capture registers       */
  T0CCR = 0;
  /* disable external match register */
  T0EMR = 0;

  /* set first sys tick interrupt    */
  /* We need to wait long enough to be sure
   * that all the init part is finished before
   * the first interrupt. Since the global
   * interrupts are enable at the end of the init
   * phase, if we miss the first one, the
   * sys_tick_handler is not called afterward
   */
  T0MR0 = 4 * sys_time.resolution_cpu_ticks;

  /* enable timer 0                  */
  T0TCR = TCR_ENABLE;

  /* select TIMER0 as IRQ    */
  VICIntSelect &= ~VIC_BIT(VIC_TIMER0);
  /* enable TIMER0 interrupt */
  VICIntEnable = VIC_BIT(VIC_TIMER0);
  /* on slot vic slot 1      */
  _VIC_CNTL(TIMER0_VIC_SLOT) = VIC_ENABLE | VIC_TIMER0;
  /* address of the ISR      */
  _VIC_ADDR(TIMER0_VIC_SLOT) = (uint32_t)TIMER0_ISR;
}


// FIXME : nb_tick rollover ???
//
// 97 days at 512hz
// 12 hours at 100khz
//
static inline void sys_tick_irq_handler(void)
{

  /* set match register for next interrupt */
  T0MR0 += sys_time.resolution_cpu_ticks - 1;

  sys_time.nb_tick++;
  sys_time.nb_sec_rem += sys_time.resolution_cpu_ticks;
  if (sys_time.nb_sec_rem >= sys_time.cpu_ticks_per_sec) {
    sys_time.nb_sec_rem -= sys_time.cpu_ticks_per_sec;
    sys_time.nb_sec++;
#ifdef SYS_TIME_LED
    LED_TOGGLE(SYS_TIME_LED);
#endif
  }
  for (unsigned int i = 0; i < SYS_TIME_NB_TIMER; i++) {
    if (sys_time.timer[i].in_use &&
        sys_time.nb_tick >= sys_time.timer[i].end_time) {
      sys_time.timer[i].end_time += sys_time.timer[i].duration;
      sys_time.timer[i].elapsed = true;
      if (sys_time.timer[i].cb) {
        sys_time.timer[i].cb(i);
      }
    }
  }
}

void TIMER0_ISR(void)
{
  ISR_ENTRY();

  while (T0IR & TIMER0_IT_MASK) {

    if (T0IR & SYS_TICK_IT) {
      sys_tick_irq_handler();
      T0IR = SYS_TICK_IT;
    }

#if defined ACTUATORS && ( defined SERVOS_4017 || defined SERVOS_4015_MAT || defined SERVOS_PPM_MAT)
    if (T0IR & ACTUATORS_IT) {
#ifdef SERVOS_4017
      SERVOS_4017_ISR();
#endif
#ifdef SERVOS_4015_MAT
      Servos4015Mat_ISR();
#endif
#ifdef SERVOS_PPM_MAT
      ServosPPMMat_ISR();
#endif
      T0IR = ACTUATORS_IT;
    }
#endif /* ACTUATORS && (SERVOS_4017 || SERVOS_4015_MAT || SERVOS_PPM_MAT) */

#if defined RADIO_CONTROL && defined RADIO_CONTROL_TYPE_PPM
    if (T0IR & PPM_IT) {
      PPM_ISR();
      T0IR = PPM_IT;
    }
#endif
#ifdef TRIGGER_EXT
    if (T0IR & TRIGGER_IT) {
      TRIG_ISR();
      T0IR = TRIGGER_IT;
      LED_TOGGLE(3);
    }
#endif
#ifdef MB_SCALE
    if (T0IR & MB_SCALE_IT) {
      MB_SCALE_ICP_ISR();
      T0IR = MB_SCALE_IT;
    }
#endif
#ifdef MB_TACHO
    if (T0IR & MB_TACHO_IT) {
      MB_TACHO_ISR();
      T0IR = MB_TACHO_IT;
    }
#endif
#ifdef USE_PWM_INPUT1
    if (T0IR & PWM_INPUT_IT1) {
      PWM_INPUT_ISR_1();
      T0IR = PWM_INPUT_IT1;
    }
#endif
#ifdef USE_PWM_INPUT2
    if (T0IR & PWM_INPUT_IT2) {
      PWM_INPUT_ISR_2();
      T0IR = PWM_INPUT_IT2;
    }
#endif
#ifdef USE_AMI601
    if (T0IR & AMI601_IT) {
      AMI601_ISR();
      T0IR = AMI601_IT;
    }
#endif
  }
  VICVectAddr = 0x00000000;
  ISR_EXIT();
}
