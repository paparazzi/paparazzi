/******************************************************************************
 *
 * $RCSfile$
 * $Revision$
 *
 * This module provides information about the project configuration
 * Copyright 2004, R O SoftWare
 * No guarantees, warrantees, or promises, implied or otherwise.
 * May be used for hobby or commercial purposes provided copyright
 * notice remains intact.
 *
 * Modified for Keil MCB2130 by Martin Thomas <mthomas@rhrk.uni-kl.de>
 *
 *****************************************************************************/

#ifndef INC_CONFIG_H
#define INC_CONFIG_H

#include "types.h"
#include "LPC21xx.h"

// some handy DEFINES
#ifndef FALSE
#define FALSE               0
#ifndef TRUE
#define TRUE                !FALSE
#endif
#endif

#ifndef BIT
#define BIT(n)              (1L << (n))
#endif

// declare functions and values from crt0.S & the linker control file
extern void reset(void);
// extern void exit(void);
extern void abort(void);
// maybe add interrupt vector addresses

#define HOST_BAUD           (115200)

#define WDOG()

// PLL setup values are computed within the LPC include file
// It relies upon the following defines
#define FOSC                (14745600)  // Master Oscillator Freq.
#define PLL_MUL             (4)         // PLL Multiplier
#define CCLK                (FOSC * PLL_MUL) // CPU Clock Freq.

// Pheripheral Bus Speed Divider
#define PBSD                2           // MUST BE 1, 2, or 4
#define PCLK                (CCLK / PBSD) // Pheripheal Bus Clock Freq.

// Do some value range testing
#if ((FOSC < 10000000) || (FOSC > 25000000))
#error Fosc out of range (10MHz-25MHz)
#error correct and recompile
#endif

#if ((CCLK < 10000000) || (CCLK > 60000000))
#error cclk out of range (10MHz-60MHz)
#error correct PLL_MUL and recompile
#endif

#if ((FCCO < 150000000) || (FCCO > 320000000))
#error Fcco out of range (156MHz-320MHz)
#error internal algorithm error
#endif

#if ((PBSD != 1) && (PBSD != 2) && (PBSD != 4))
#error Pheripheal Bus Speed Divider (PBSD) illegal value (1, 2, or 4)
#endif

// Port Bit Definitions & Macros:    Description - initial conditions

// The following defines are for the Olimex (LPC2138)
// PIO 0
#define TXD0_BIT              BIT(0)      // used by UART0
#define RXD0_BIT              BIT(1)      // used by UART0
#define P0_02_UNUSED_BIT      BIT(2)      // P0.02 unused - low output
#define P0_03_UNUSED_BIT      BIT(3)      // P0.03 unused - low output
#define P0_04_UNUSED_BIT      BIT(4)      // P0.04 unused - low output
#define P0_05_UNUSED_BIT      BIT(5)      // P0.05 unused - low output
#define PPM_IN_BIT            BIT(6)      // P0.06 unused - low output
//#define P0_07_UNUSED_BIT      BIT(7)      // P0.07 unused - low output
#define SERV1_CLOCK_BIT       BIT(7)      // P0.07 unused - low output
#define TXD1_BIT              BIT(8)      // used by UART1
#define RXD1_BIT              BIT(9)      // used by UART1
//#define P0_08_UNUSED_BIT      BIT(8)      // P0.08 unused - low output
//#define P0_09_UNUSED_BIT      BIT(9)      // P0.09 unused - low output
#define P0_10_UNUSED_BIT      BIT(10)     // P0.10 unused - low output
#define P0_11_UNUSED_BIT      BIT(11)     // P0.11 unused - low output
#define LED1_BIT              BIT(12)     // P0.12 LED1 low active
#define LED2_BIT              BIT(13)     // P0.13 LED1 low active
#define P0_14_UNUSED_BIT      BIT(14)     // P0.14 unused - low output
#define SW1_BIT               BIT(15)     // P0.15 Switch 1 - active low input
#define SW2_BIT               BIT(16)     // P0.16 Switch 2 - active low input
#define P0_17_UNUSED_BIT      BIT(17)     // P0.17 unused - low output
#define P0_18_UNUSED_BIT      BIT(18)     // P0.18 unused - low output
#define P0_19_UNUSED_BIT      BIT(19)     // P0.19 unused - low output
#define P0_20_UNUSED_BIT      BIT(20)     // P0.20 unused - low output
#define SERV0_CLOCK_BIT       BIT(21)     // P0.21 unused - low output
#define P0_22_UNUSED_BIT      BIT(22)     // P0.22 unused - low output
#define P0_23_UNUSED_BIT      BIT(23)     // P0.23 unused - low output
#define P0_24_UNUSED_BIT      BIT(24)     // P0.24 unused - low output
#define P0_25_UNUSED_BIT      BIT(25)     // P0.25 unused - low output
#define P0_26_UNUSED_BIT      BIT(26)     // P0.26 unused - low output
#define P0_27_UNUSED_BIT      BIT(27)     // P0.27 unused - low output
#define P0_28_UNUSED_BIT      BIT(28)     // P0.28 unused - low output
#define P0_29_UNUSED_BIT      BIT(29)     // P0.29 unused - low output
#define P0_30_UNUSED_BIT      BIT(30)     // P0.30 unused - low output
#define P0_31_UNUSED_BIT      BIT(31)     // P0.31 unused - low output

// PIO 1
#define P1_00_UNUSED_BIT      BIT(0)      // P1.00 unused - low output
#define P1_01_UNUSED_BIT      BIT(1)      // P1.00 unused - low output
#define P1_02_UNUSED_BIT      BIT(2)      // P1.02 unused - low output
#define P1_03_UNUSED_BIT      BIT(3)      // P1.03 unused - low output
#define P1_04_UNUSED_BIT      BIT(4)      // P1.04 unused - low output
#define P1_05_UNUSED_BIT      BIT(5)      // P1.05 unused - low output
#define P1_06_UNUSED_BIT      BIT(6)      // P1.06 unused - low output
#define P1_07_UNUSED_BIT      BIT(7)      // P1.07 unused - low output
#define P1_08_UNUSED_BIT      BIT(8)      // P1.08 unused - low output
#define P1_09_UNUSED_BIT      BIT(9)      // P1.09 unused - low output
#define P1_10_UNUSED_BIT      BIT(10)     // P1.10 unused - low output
#define P1_11_UNUSED_BIT      BIT(11)     // P1.11 unused - low output
#define P1_12_UNUSED_BIT      BIT(12)     // P1.12 unused - low output
#define P1_13_UNUSED_BIT      BIT(13)     // P1.13 unused - low output
#define P1_14_UNUSED_BIT      BIT(14)     // P1.14 unused - low output
#define P1_15_UNUSED_BIT      BIT(15)     // P1.15 unused - low output
#define P1_16_UNUSED_BIT      BIT(16)     // P1.16 unused - low output
#define P1_17_UNUSED_BIT      BIT(17)     // P1.17 unused - low output
#define P1_18_UNUSED_BIT      BIT(18)     // P1.18 unused - low output
#define P1_19_UNUSED_BIT      BIT(19)     // P1.19 unused - low output
#define SERV0_DATA_BIT        BIT(20)     // P1.20 unused - low output
#define SERV0_RESET_BIT       BIT(21)     // P1.21 unused - low output
#define P1_22_UNUSED_BIT      BIT(22)     // P1.22 unused - low output
#define P1_23_UNUSED_BIT      BIT(23)     // P1.23 unused - low output
#define P1_24_UNUSED_BIT      BIT(24)     // P1.24 unused - low output
#define P1_25_UNUSED_BIT      BIT(25)     // P1.25 unused - low output
#define P1_26_UNUSED_BIT      BIT(26)     // P1.26 unused - low output
#define P1_27_UNUSED_BIT      BIT(27)     // P1.27 unused - low output
#define P1_28_UNUSED_BIT      BIT(28)     // P1.28 unused - low output
#define SERV1_RESET_BIT       BIT(29)     // P1.29 unused - low output
#define SERV1_DATA_BIT        BIT(30)     // P1.30 unused - low output
#define P1_31_UNUSED_BIT      BIT(31)     // P1.31 unused - low output



#define PIO0_INPUT_BITS      (uint32_t) ( \
                                         SW1_BIT | \
                                         SW2_BIT | \
					 0 )

#define PIO0_ZERO_BITS       (uint32_t) ( \
                                         P0_02_UNUSED_BIT | \
                                         P0_03_UNUSED_BIT | \
                                         P0_04_UNUSED_BIT | \
                                         P0_05_UNUSED_BIT | \
					 SERV1_CLOCK_BIT  | \
	                                 P0_10_UNUSED_BIT | \
					 P0_11_UNUSED_BIT | \
					 LED1_BIT         | \
					 LED2_BIT         | \
					 P0_14_UNUSED_BIT | \
					 P0_17_UNUSED_BIT | \
					 P0_18_UNUSED_BIT | \
					 P0_19_UNUSED_BIT | \
					 P0_20_UNUSED_BIT | \
					 SERV0_CLOCK_BIT  | \
					 P0_22_UNUSED_BIT | \
					 P0_23_UNUSED_BIT | \
                                         P0_24_UNUSED_BIT | \
                                         P0_25_UNUSED_BIT | \
                                         P0_26_UNUSED_BIT | \
                                         P0_27_UNUSED_BIT | \
                                         P0_28_UNUSED_BIT | \
                                         P0_29_UNUSED_BIT | \
                                         P0_30_UNUSED_BIT | \
					 P0_31_UNUSED_BIT | \
                                         0 )

#define PIO0_ONE_BITS        (uint32_t) ( \
                                         0 )

#define PIO0_OUTPUT_BITS     (uint32_t) ( \
                                         PIO0_ZERO_BITS | \
                                         PIO0_ONE_BITS )



#define PIO1_INPUT_BITS      (uint32_t) ( \
                                         0 )

#define PIO1_ZERO_BITS       (uint32_t) ( \
                                         P1_00_UNUSED_BIT | \
					 P1_01_UNUSED_BIT | \
					 P1_02_UNUSED_BIT | \
					 P1_03_UNUSED_BIT | \
					 P1_04_UNUSED_BIT | \
					 P1_05_UNUSED_BIT | \
					 P1_06_UNUSED_BIT | \
					 P1_07_UNUSED_BIT | \
					 P1_08_UNUSED_BIT | \
					 P1_09_UNUSED_BIT | \
					 P1_10_UNUSED_BIT | \
					 P1_11_UNUSED_BIT | \
					 P1_12_UNUSED_BIT | \
					 P1_13_UNUSED_BIT | \
					 P1_14_UNUSED_BIT | \
					 P1_15_UNUSED_BIT | \
					 P1_16_UNUSED_BIT | \
					 P1_17_UNUSED_BIT | \
					 P1_18_UNUSED_BIT | \
					 P1_19_UNUSED_BIT | \
					 SERV0_DATA_BIT   | \
					 SERV0_RESET_BIT  | \
					 P1_22_UNUSED_BIT | \
					 P1_23_UNUSED_BIT | \
					 P1_24_UNUSED_BIT | \
					 P1_25_UNUSED_BIT | \
					 P1_26_UNUSED_BIT | \
					 P1_27_UNUSED_BIT | \
					 P1_28_UNUSED_BIT | \
					 SERV1_RESET_BIT  | \
					 SERV1_DATA_BIT   | \
					 P1_31_UNUSED_BIT | \
                                         0 )

#define PIO1_ONE_BITS        (uint32_t) ( \
                                         0 )

#define PIO1_OUTPUT_BITS     (uint32_t) ( \
                                         PIO1_ZERO_BITS | \
                                         PIO1_ONE_BITS )

#endif
