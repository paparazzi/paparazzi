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
  mcu_reboot(MCU_REBOOT_FAST);
}

CH_IRQ_HANDLER(NMI_Handler)
{
  hard_fault = true;
  mcu_reboot(MCU_REBOOT_FAST);
}

CH_IRQ_HANDLER(MemManage_Handler)
{
  hard_fault = true;
  mcu_reboot(MCU_REBOOT_FAST);
}

CH_IRQ_HANDLER(BusFault_Handler)
{
  hard_fault = true;
  mcu_reboot(MCU_REBOOT_FAST);
}

CH_IRQ_HANDLER(UsageFault_Handler)
{
  hard_fault = true;
  mcu_reboot(MCU_REBOOT_FAST);
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

#endif /* USE_HARD_FAULT_RECOVERY */

/**
 * @brief RTC backup register values
 */
enum rtc_boot_magic {
  RTC_BOOT_OFF  = 0,            ///< Normal boot
  RTC_BOOT_HOLD = 0xb0070001,   ///< Hold in bootloader, do not boot application
  RTC_BOOT_FAST = 0xb0070002,   ///< No timeout in bootloader
  RTC_BOOT_CANBL = 0xb0080000,  ///< CAN bootloader, ORd with 8 bit local node ID
  RTC_BOOT_FWOK = 0xb0093a26    ///< indicates FW ran for 30s
};

/* Local functions */
static void mcu_deep_sleep(void);
#if defined(USE_RTC_BACKUP)
static void mcu_set_rtcbackup(uint32_t val);
#endif

/**
 * @brief Initialize the specific archittecture functions
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
 * @brief Reboot the MCU
 * - POWEROFF will go the deep sleep
 * - NORMAL will do a normal reboot (also to bootloader)
 * - FAST will try to skip the bootloader
 * - BOOTLOADER will try to keep the MCU in bootloader
 * @param reboot_state The sate to reboot to
 */
void mcu_reboot(enum reboot_state_t reboot_state)
{
  // Powering off/deep sleep instead
  if(reboot_state == MCU_REBOOT_POWEROFF) {
    mcu_deep_sleep();
    return;
  }

#if defined(USE_RTC_BACKUP)
  // Set the RTC backup register if possible
  if(reboot_state == MCU_REBOOT_FAST)
    mcu_set_rtcbackup(RTC_BOOT_FAST);
  else if(reboot_state == MCU_REBOOT_BOOTLOADER)
    mcu_set_rtcbackup(RTC_BOOT_HOLD);
#endif

  // Restart the MCU
  NVIC_SystemReset();
}

/**
 * @brief Save energy for performing operations on shutdown
 * Used for example to shutdown SD-card logging
 */
void mcu_energy_save(void)
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

/** Put MCU into deep sleep mode
 *
 *  This can be used when closing the SD log files
 *  right after a power down to save the remaining
 *  energy for the SD card internal MCU
 *
 *  Never call this during flight!
 */
static void mcu_deep_sleep(void)
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

#if defined(USE_RTC_BACKUP)
/**
 * @brief Set the RTC backup register
 * This sets the first register in the RTC backup register, used for bootloader
 * @param val The value to set in the register
 */
static void mcu_set_rtcbackup(uint32_t val) {
#if !defined(STM32F1)
  if ((RCC->BDCR & RCC_BDCR_RTCEN) == 0) {
    RCC->BDCR |= STM32_RTCSEL;
    RCC->BDCR |= RCC_BDCR_RTCEN;
  }
#ifdef PWR_CR_DBP
  PWR->CR |= PWR_CR_DBP;
#else
  PWR->CR1 |= PWR_CR1_DBP;
#endif
#endif

#if defined(STM32F1)
  volatile uint32_t *dr = (volatile uint32_t *)&BKP->DR1;
  dr[0] = (val) & 0xFFFF;
  dr[1] = (val) >> 16;
#elif defined(STM32G4)
  ((volatile uint32_t *)&TAMP->BKP0R)[0] = val;
#else
  ((volatile uint32_t *)&RTC->BKP0R)[0] = val;
#endif
}
#endif /* USE_RTC_BACKUP */
