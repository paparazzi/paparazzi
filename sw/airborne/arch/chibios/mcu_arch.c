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

#include "mcu_periph/ram_arch.h"

#if defined(STM32H7XX)
typedef struct {
  uint32_t              *init_text_area;
  uint32_t              *init_area;
  uint32_t              *clear_area;
  uint32_t              *no_init_area;
} ram_init_area_t;

static void initRam0nc(void);
static void init_ram_areas(const ram_init_area_t *rap);
static void mpuConfigureNonCachedRam(void);
#endif

#if USE_HARD_FAULT_RECOVERY

#if defined(STM32F4XX) || defined (STM32F7XX)
#define BCKP_SECTION ".ram5"
#define IN_BCKP_SECTION(var) var __attribute__ ((section(BCKP_SECTION), aligned(8)))
#elif defined(STM32H7XX)
#define BCKP_SECTION ".ram7"
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
#define __PWR_BACKUP_REG PWR->CSR
#define __PWR_BACKUP_ENABLE PWR_CSR_BRE
#define __PWR_BACKUP_READY PWR_CSR_BRR
#define __RCC_RESET_REGISTER RCC->CSR
#define __RCC_RESET_FLAG RCC_CSR_SFTRSTF
#define __RCC_RESET_REMOVE_FLAG RCC_CSR_RMVF
#elif defined(STM32F7XX)
#define __PWR_BACKUP_REG PWR->CSR1
#define __PWR_BACKUP_ENABLE PWR_CSR1_BRE
#define __PWR_BACKUP_READY PWR_CSR1_BRR
#define __RCC_RESET_REGISTER RCC->CSR
#define __RCC_RESET_FLAG RCC_CSR_SFTRSTF
#define __RCC_RESET_REMOVE_FLAG RCC_CSR_RMVF
#elif defined(STM32H7XX)
#define __PWR_BACKUP_REG PWR->CR2
#define __PWR_BACKUP_ENABLE PWR_CR2_BREN
#define __PWR_BACKUP_READY PWR_CR2_BRRDY
#define __RCC_RESET_REGISTER RCC->RSR
#define __RCC_RESET_FLAG RCC_RSR_SFTRSTF
#define __RCC_RESET_REMOVE_FLAG RCC_RSR_RMVF
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

#if defined(STM32H7XX)
  mpuConfigureNonCachedRam();
#endif

#if USE_HARD_FAULT_RECOVERY
  /* Backup domain SRAM enable, and with it, the regulator */
#if defined(STM32F4XX) || defined(STM32F7XX)
  RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;
  __PWR_BACKUP_REG |= __PWR_BACKUP_ENABLE;
  while ((__PWR_BACKUP_REG & __PWR_BACKUP_READY) == 0) ; /* Waits until the regulator is stable */
#endif /* STM32F4 | STM32F7 */

  // test if last reset was a 'real' hard fault
  recovering_from_hard_fault = false;
  if (!(__RCC_RESET_REGISTER & __RCC_RESET_FLAG)) {
    // not coming from soft reset
    hard_fault = false;
  } else if ((__RCC_RESET_REGISTER & __RCC_RESET_FLAG) && !hard_fault) {
    // this is a soft reset, probably from a debug probe, so let's start in normal mode
    hard_fault = false;
  } else {
    // else real hard fault
    recovering_from_hard_fault = true;
    hard_fault = false;
  }
  // *MANDATORY* clear of rcc bits
  __RCC_RESET_REGISTER = __RCC_RESET_REMOVE_FLAG;
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



#if defined(STM32H7XX)
/*
  nocache regions are 
    ° ram0nc for sdmmc1
    ° ram3 for miscellanous ?
    ° ram4 for bdma attached peripherals (i2c4, spi6, adc3)
 */

extern const uint32_t __ram0nc_base__;
extern const uint32_t __ram0nc_size__;
extern const uint32_t __ram3_base__;
extern const uint32_t __ram3_size__;
extern const uint32_t __ram4_base__;
extern const uint32_t __ram4_size__;

static uint32_t getMPU_RASR_SIZE(const uint32_t ldSize)
{
  // 2^n -> n-1
  chDbgAssert(__builtin_popcount(ldSize) == 1U, "MPU region size must be 2^n");
  chDbgAssert(ldSize >= 32U, "MPU region size must be >= 32");
  return MPU_RASR_SIZE(__builtin_ctz(ldSize) - 1U);
}


static void mpuConfigureNonCachedRam(void)
{
  const uint32_t mpuSharedOption = MPU_RASR_ATTR_AP_RW_RW |
    MPU_RASR_ATTR_NON_CACHEABLE | MPU_RASR_ATTR_S |
    MPU_RASR_ENABLE;

  const uint32_t ram0nc_base = (uint32_t) &__ram0nc_base__;
  const uint32_t ram3_base = (uint32_t) &__ram3_base__;
  const uint32_t ram4_base = (uint32_t) &__ram4_base__;

  const uint32_t ram0nc_size = (uint32_t) &__ram0nc_size__;
  const uint32_t ram3_size = (uint32_t) &__ram3_size__;
  const uint32_t ram4_size = (uint32_t) &__ram4_size__;

  chDbgAssert(ram0nc_base == 0x24000000, "MPU ram0nc addr mismatch");
  chDbgAssert(ram3_base == 0x30040000, "MPU ram3 addr mismatch");
  chDbgAssert(ram4_base == 0x38000000, "MPU ram4 addr mismatch");

  chDbgAssert((ram0nc_base % ram0nc_size) == 0, "MPU ram0nc base addr must be size aligned");
  chDbgAssert(ram0nc_size == 128 * 1024, "MPU ram0nc size must be 128K");
  chDbgAssert((ram3_base % ram3_size) == 0, "MPU ram3 base addr must be size aligned");
  chDbgAssert((ram4_base % ram4_size) == 0, "MPU ram4 base addr must be size aligned");
  chDbgAssert(getMPU_RASR_SIZE(ram0nc_size) == MPU_RASR_SIZE_128K, "getMPU_RASR_SIZE error");

  
  mpuConfigureRegion(MPU_REGION_6,
		     ram0nc_base,
		     getMPU_RASR_SIZE(ram0nc_size) | mpuSharedOption
		     );
  mpuConfigureRegion(MPU_REGION_5,
		     ram3_base,
		     getMPU_RASR_SIZE(ram3_size) | mpuSharedOption
		     );
  mpuConfigureRegion(MPU_REGION_4,
		     ram4_base,
		     getMPU_RASR_SIZE(ram4_size) | mpuSharedOption
		     );
  initRam0nc();
  mpuEnable(MPU_CTRL_PRIVDEFENA);
  __ISB();
  __DSB();
   SCB_CleanInvalidateDCache();

}

static void initRam0nc(void)
{
  extern uint32_t __ram0nc_init_text__, __ram0nc_init__, __ram0nc_clear__, __ram0nc_noinit__;
  static const ram_init_area_t ram_areas[1] = {
    {&__ram0nc_init_text__, &__ram0nc_init__, &__ram0nc_clear__, &__ram0nc_noinit__},
  };
  init_ram_areas(ram_areas);
  
}

static void init_ram_areas(const ram_init_area_t *rap)
{
  uint32_t *tp = rap->init_text_area;
  uint32_t *p = rap->init_area;

  /* Copying initialization data.*/
  while (p < rap->clear_area) {
    *p = *tp;
    p++;
    tp++;
  }
  
  /* Zeroing clear area.*/
  while (p < rap->no_init_area) {
    *p = 0;
    p++;
  }
}

#endif
