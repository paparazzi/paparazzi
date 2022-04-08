/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
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
 * @file arch/chibios/mcu_arch.h
 * Microcontroller initialization function for ChibiOS
 *
 */
#ifndef CHIBIOS_MCU_ARCH_H
#define CHIBIOS_MCU_ARCH_H

#include "std.h"

extern void mcu_arch_init(void);

#if USE_HARD_FAULT_RECOVERY
extern bool recovering_from_hard_fault;
#endif

#include <ch.h>

/** Put MCU into deep sleep mode
 *
 *  This can be used when closing the SD log files
 *  right after a power down to save the remaining
 *  energy for the SD card internal MCU
 *
 *  Never call this during flight!
 */
static inline void mcu_deep_sleep(void)
{
#if defined(STM32F4XX)
  /* clear PDDS and LPDS bits */
  PWR->CR &= ~(PWR_CR_PDDS | PWR_CR_LPDS);
  /* set LPDS and clear  */
  PWR->CR |= (PWR_CR_LPDS | PWR_CR_CSBF | PWR_CR_CWUF);
#elif defined(STM32F7XX)
  /* clear PDDS and LPDS bits */
  PWR->CR1 &= ~(PWR_CR1_PDDS | PWR_CR1_LPDS);
  /* set LPDS and clear  */
  PWR->CR1 |= (PWR_CR1_LPDS | PWR_CR1_CSBF);
#elif defined(STM32H7XX)
  /* clear LPDS */
  PWR->CR1 &= ~PWR_CR1_LPDS;
  /* set LPDS */
  PWR->CR1 |= PWR_CR1_LPDS;
#endif

  /* Setup the deepsleep mask */
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  __disable_irq();

  __SEV();
  __WFE();
  __WFE();

  __enable_irq();

  /* clear the deepsleep mask */
  SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
}

/** Request a software reset of the MCU
 */
static inline void mcu_reset(void)
{
  NVIC_SystemReset();
}

/** Call board specific energy saving
 *  Can be necessary for closing on power off
 */
extern void mcu_periph_energy_save(void);

#endif /* CHIBIOS_MCU_ARCH_H */
