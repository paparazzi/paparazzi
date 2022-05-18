/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
 * 2016 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * 2016 Alexandre Bustico <alexandre.bustico@enac.fr>
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
 * @file arch/chibios/mcu_arch.c
 * Microcontroller initialization function for ChibiOS
 *
 * ChibiOS initialized peripherals by itself, only interrupt
 * vector has to be relocated for Luftboot
 */

/* ChibiOS includes */
#include "ch.h"
#include "hal.h"
/* Paparazzi includes */
#include "mcu.h"

#if USE_HARD_FAULT_RECOVERY

#if defined(STM32F4XX) || defined (STM32F7XX)
#define BCKP_SECTION ".ram5"
#define IN_BCKP_SECTION(var) var __attribute__ ((section(BCKP_SECTION), aligned(8)))
#else
#error "No backup ram available"
#endif
IN_BCKP_SECTION(volatile bool hard_fault);

/*
 * Set hard fault handlers to trigger a soft reset
 * This will set a flag that can be tested at startup
 */

CH_IRQ_HANDLER(HardFault_Handler)
{
  hard_fault = true;
  NVIC_SystemReset();
}

CH_IRQ_HANDLER(NMI_Handler)
{
  hard_fault = true;
  NVIC_SystemReset();
}

CH_IRQ_HANDLER(MemManage_Handler)
{
  hard_fault = true;
  NVIC_SystemReset();
}

CH_IRQ_HANDLER(BusFault_Handler)
{
  hard_fault = true;
  NVIC_SystemReset();
}

CH_IRQ_HANDLER(UsageFault_Handler)
{
  hard_fault = true;
  NVIC_SystemReset();
}

bool recovering_from_hard_fault;

// select correct register
#if defined(STM32F4XX)
#define __PWR_CSR PWR->CSR
#define __PWR_CSR_BRE PWR_CSR_BRE
#define __PWR_CSR_BRR PWR_CSR_BRR
#elif defined(STM32F7XX)
#define __PWR_CSR PWR->CSR1
#define __PWR_CSR_BRE PWR_CSR1_BRE
#define __PWR_CSR_BRR PWR_CSR1_BRR
#else
#error Hard fault recovery not supported
#endif

#endif


/*
 * SCB_VTOR has to be relocated if Luftboot is used
 * The new SCB_VTOR location is defined in the board makefile
 */
void mcu_arch_init(void)
{
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

#if USE_HARD_FAULT_RECOVERY
  /* Backup domain SRAM enable, and with it, the regulator */
#if defined(STM32F4XX) || defined(STM32F7XX)
  RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;
  __PWR_CSR |= __PWR_CSR_BRE;
  while ((__PWR_CSR & __PWR_CSR_BRR) == 0) ; /* Waits until the regulator is stable */
#endif /* STM32F4 | STM32F7 */

  // test if last reset was a 'real' hard fault
  recovering_from_hard_fault = false;
  if (!(RCC->CSR & RCC_CSR_SFTRSTF)) {
    // not coming from soft reset
    hard_fault = false;
  } else if ((RCC->CSR & RCC_CSR_SFTRSTF) && !hard_fault) {
    // this is a soft reset, probably from a debug probe, so let's start in normal mode
    hard_fault = false;
  } else {
    // else real hard fault
    recovering_from_hard_fault = true;
    hard_fault = false;
  }
  // *MANDATORY* clear of rcc bits
  RCC->CSR = RCC_CSR_RMVF;
  // end of reset bit probing
#endif /* USE_HARD_FAULT_RECOVERY */

}

/**
 * @brief Save energy for performing operations on shutdown
 * Used for example to shutdown SD-card logging
 */
void mcu_periph_energy_save(void)
{
#if defined(ENERGY_SAVE_INPUTS)
  BOARD_GROUP_DECLFOREACH(input_line, ENERGY_SAVE_INPUTS) {
    palSetLineMode(input_line, PAL_MODE_INPUT);
  }
#endif
#if defined(ENERGY_SAVE_LOWS)
  BOARD_GROUP_DECLFOREACH(input_low, ENERGY_SAVE_LOWS) {
    palClearLine(input_low);
  }
#endif
}

