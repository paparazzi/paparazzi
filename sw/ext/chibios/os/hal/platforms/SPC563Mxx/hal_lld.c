/*
    SPC5 HAL - Copyright (C) 2013 STMicroelectronics

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    SPC563Mxx/hal_lld.c
 * @brief   SPC563Mxx HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "ch.h"
#include "hal.h"

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {
  uint32_t n;

  /* FLASH wait states and prefetching setup.*/
  CFLASH0.BIUCR.R  = SPC5_FLASH_BIUCR | SPC5_FLASH_WS;
  CFLASH0.BIUCR2.R = 0;
  CFLASH0.PFCR3.R  = 0;

  /* Optimal crossbar settings. The DMA priority is placed above the CPU
     priority in order to not starve I/O activities while the CPU is
     executing tight loops (FLASH and SRAM slave ports only).
     The SRAM is parked on the load/store port, for some unknown reason it
     is defaulted on the instructions port and this kills performance.*/
  XBAR.SGPCR3.B.PARK = 4;               /* RAM slave on load/store port.*/
  XBAR.MPR0.R = 0x00030201;             /* Flash slave port priorities:
                                            eDMA (1):              0 (highest)
                                            Core Instructions (0): 1
                                            Undocumented (2):      2
                                            Core Data (4):         3        */
  XBAR.MPR3.R = 0x00030201;             /* SRAM slave port priorities:
                                            eDMA (1):              0 (highest)
                                            Core Instructions (0): 1
                                            Undocumented (2):      2
                                            Core Data (4):         3        */

  /* Downcounter timer initialized for system tick use, TB enabled for debug
     and measurements.*/
  n = SPC5_SYSCLK / CH_FREQUENCY;
  asm volatile ("li      %%r3, 0            \t\n"
                "mtspr   284, %%r3          \t\n"   /* Clear TBL register.  */
                "mtspr   285, %%r3          \t\n"   /* Clear TBU register.  */
                "mtspr   22, %[n]           \t\n"   /* Init. DEC register.  */
                "mtspr   54, %[n]           \t\n"   /* Init. DECAR register.*/
                "li      %%r3, 0x4000       \t\n"   /* TBEN bit.            */
                "mtspr   1008, %%r3         \t\n"   /* HID0 register.       */
                "lis     %%r3, 0x0440       \t\n"   /* DIE ARE bits.        */
                "mtspr   340, %%r3"                 /* TCR register.        */
                : : [n] "r" (n) : "r3");

  /* INTC initialization, software vector mode, 4 bytes vectors, starting
     at priority 0.*/
  INTC.MCR.R   = 0;
  INTC.CPR.R   = 0;
  INTC.IACKR.R = (uint32_t)_vectors;
}

/**
 * @brief   SPC563 clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h and
 *          @p hal_lld.h
 * @note    This function must be invoked only after the system reset.
 *
 * @special
 */
void spc_clock_init(void) {

#if !SPC5_NO_INIT
  /* PLL activation.*/
  FMPLL.ESYNCR1.B.EMODE     = 1;                    /* Enhanced mode on.    */
  FMPLL.ESYNCR1.B.CLKCFG   &= 1;                    /* Bypass mode, PLL off.*/
#if !SPC5_CLK_BYPASS
  FMPLL.ESYNCR1.B.CLKCFG   |= 2;                    /* PLL on.              */
  FMPLL.ESYNCR1.B.EPREDIV   = SPC5_CLK_PREDIV;
  FMPLL.ESYNCR1.B.EMFD      = SPC5_CLK_MFD;
  FMPLL.ESYNCR2.B.ERFD      = SPC5_CLK_RFD;
  while (!FMPLL.SYNSR.B.LOCK)
    ;
  FMPLL.ESYNCR1.B.CLKCFG   |= 4;                    /* Clock from the PLL.  */
#endif /* !SPC5_CLK_BYPASS */
#endif /* !SPC5_NO_INIT */
}

/** @} */
