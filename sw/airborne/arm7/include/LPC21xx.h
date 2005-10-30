/******************************************************************************
 *
 * $RCSfile$
 * $Revision$
 *
 * Header file for Philips LPC21xx ARM Processors 
 * Copyright 2004 R O SoftWare
 *
 * No guarantees, warrantees, or promises, implied or otherwise.
 * May be used for hobby or commercial purposes provided copyright
 * notice remains intact.
 *
 *****************************************************************************/
#ifndef INC_LPC21xx_H
#define INC_LPC21xx_H


#define REG_8           volatile unsigned char
#define REG16           volatile unsigned short
#define REG32           volatile unsigned long

#include "lpcWD.h"
#include "lpcTMR.h"
#include "lpcUART.h"
#include "lpcI2C.h"
#include "lpcSPI.h"
#include "lpcRTC.h"
#include "lpcGPIO.h"
#include "lpcPIN.h"
#include "lpcADC.h"
#include "lpcSCB.h"
#include "lpcVIC.h"

///////////////////////////////////////////////////////////////////////////////
// Watchdog
#define WD              ((wdRegs_t *)0xE0000000)

// Watchdog Registers
#define WDMOD           WD->mod         /* Watchdog Mode Register */
#define WDTC            WD->tc          /* Watchdog Time Constant Register */
#define WDFEED          WD->feed        /* Watchdog Feed Register */
#define WDTV            WD->tv          /* Watchdog Time Value Register */

///////////////////////////////////////////////////////////////////////////////
// Timer 0
#define TMR0            ((pwmTmrRegs_t *)0xE0004000)

// Timer 0 Registers
#define T0IR            TMR0->ir        /* Interrupt Register */
#define T0TCR           TMR0->tcr       /* Timer Control Register */
#define T0TC            TMR0->tc        /* Timer Counter */
#define T0PR            TMR0->pr        /* Prescale Register */
#define T0PC            TMR0->pc        /* Prescale Counter Register */
#define T0MCR           TMR0->mcr       /* Match Control Register */
#define T0MR0           TMR0->mr0       /* Match Register 0 */
#define T0MR1           TMR0->mr1       /* Match Register 1 */
#define T0MR2           TMR0->mr2       /* Match Register 2 */
#define T0MR3           TMR0->mr3       /* Match Register 3 */
#define T0CCR           TMR0->ccr       /* Capture Control Register */
#define T0CR0           TMR0->cr0       /* Capture Register 0 */
#define T0CR1           TMR0->cr1       /* Capture Register 1 */
#define T0CR2           TMR0->cr2       /* Capture Register 2 */
#define T0CR3           TMR0->cr3       /* Capture Register 3 */
#define T0EMR           TMR0->emr       /* External Match Register */

///////////////////////////////////////////////////////////////////////////////
// Timer 1
#define TMR1            ((pwmTmrRegs_t *)0xE0008000)

// Timer 1 Registers
#define T1IR            TMR1->ir        /* Interrupt Register */
#define T1TCR           TMR1->tcr       /* Timer Control Register */
#define T1TC            TMR1->tc        /* Timer Counter */
#define T1PR            TMR1->pr        /* Prescale Register */
#define T1PC            TMR1->pc        /* Prescale Counter Register */
#define T1MCR           TMR1->mcr       /* Match Control Register */
#define T1MR0           TMR1->mr0       /* Match Register 0 */
#define T1MR1           TMR1->mr1       /* Match Register 1 */
#define T1MR2           TMR1->mr2       /* Match Register 2 */
#define T1MR3           TMR1->mr3       /* Match Register 3 */
#define T1CCR           TMR1->ccr       /* Capture Control Register */
#define T1CR0           TMR1->cr0       /* Capture Register 0 */
#define T1CR1           TMR1->cr1       /* Capture Register 1 */
#define T1CR2           TMR1->cr2       /* Capture Register 2 */
#define T1CR3           TMR1->cr3       /* Capture Register 3 */
#define T1EMR           TMR1->emr       /* External Match Register */

///////////////////////////////////////////////////////////////////////////////
// Pulse Width Modulator (PWM)
#define PWM             ((pwmTmrRegs_t *)0xE0014000)

// PWM Registers
#define PWMIR           PWM->ir         /* Interrupt Register */
#define PWMTCR          PWM->tcr        /* Timer Control Register */
#define PWMTC           PWM->tc         /* Timer Counter */
#define PWMPR           PWM->pr         /* Prescale Register */
#define PWMPC           PWM->pc         /* Prescale Counter Register */
#define PWMMCR          PWM->mcr        /* Match Control Register */
#define PWMMR0          PWM->mr0        /* Match Register 0 */
#define PWMMR1          PWM->mr1        /* Match Register 1 */
#define PWMMR2          PWM->mr2        /* Match Register 2 */
#define PWMMR3          PWM->mr3        /* Match Register 3 */
#define PWMMR4          PWM->mr4        /* Match Register 4 */
#define PWMMR5          PWM->mr5        /* Match Register 5 */
#define PWMMR6          PWM->mr6        /* Match Register 6 */
#define PWMPCR          PWM->pcr        /* Control Register */
#define PWMLER          PWM->ler        /* Latch Enable Register */

///////////////////////////////////////////////////////////////////////////////
// Universal Asynchronous Receiver Transmitter 0 (UART0)
#define UART0           ((uartRegs_t *)0xE000C000)
#define U0_PINSEL       (0x00000005)    /* PINSEL0 Value for UART0 */
#define U0_PINMASK      (0x0000000F)    /* PINSEL0 Mask for UART0 */

// UART0 Registers
#define U0RBR           UART0->rbr      /* Receive Buffer Register */
#define U0THR           UART0->thr      /* Transmit Holding Register */
#define U0IER           UART0->ier      /* Interrupt Enable Register */
#define U0IIR           UART0->iir      /* Interrupt ID Register */
#define U0FCR           UART0->fcr      /* FIFO Control Register */
#define U0LCR           UART0->lcr      /* Line Control Register */
#define U0LSR           UART0->lsr      /* Line Status Register */
#define U0SCR           UART0->scr      /* Scratch Pad Register */
#define U0DLL           UART0->dll      /* Divisor Latch Register (LSB) */
#define U0DLM           UART0->dlm      /* Divisor Latch Register (MSB) */

///////////////////////////////////////////////////////////////////////////////
// Universal Asynchronous Receiver Transmitter 1 (UART1)
#define UART1           ((uartRegs_t *)0xE0010000)
#define U1_PINSEL       (0x00050000)    /* PINSEL0 Value for UART1 */
#define U1_PINMASK      (0x000F0000)    /* PINSEL0 Mask for UART1 */

// UART1 Registers
#define U1RBR           UART1->rbr      /* Receive Buffer Register */
#define U1THR           UART1->thr      /* Transmit Holding Register */
#define U1IER           UART1->ier      /* Interrupt Enable Register */
#define U1IIR           UART1->iir      /* Interrupt ID Register */
#define U1FCR           UART1->fcr      /* FIFO Control Register */
#define U1LCR           UART1->lcr      /* Line Control Register */
#define U1MCR           UART1->mcr      /* MODEM Control Register */
#define U1LSR           UART1->lsr      /* Line Status Register */
#define U1MSR           UART1->msr      /* MODEM Status Register */
#define U1SCR           UART1->scr      /* Scratch Pad Register */
#define U1DLL           UART1->dll      /* Divisor Latch Register (LSB) */
#define U1DLM           UART1->dlm      /* Divisor Latch Register (MSB) */

///////////////////////////////////////////////////////////////////////////////
// I2C Interface
#define I2C             ((i2cRegs_t *)0xE001C000)

// I2C Registers
#define I2CONSET        I2C->conset     /* Control Set Register */
#define I2STAT          I2C->stat       /* Status Register */
#define I2DAT           I2C->dat        /* Data Register */
#define I2ADR           I2C->adr        /* Slave Address Register */
#define I2SCLH          I2C->sclh       /* SCL Duty Cycle Register (high half word) */
#define I2SCLL          I2C->scll       /* SCL Duty Cycle Register (low half word) */
#define I2CONCLR        I2C->conclr     /* Control Clear Register */

///////////////////////////////////////////////////////////////////////////////
// Serial Peripheral Interface 0 (SPI0)
#define SPI0            ((spiRegs_t *)0xE0020000)

// SPI0 Registers
#define S0SPCR          SPI0->cr        /* Control Register */
#define S0SPSR          SPI0->sr        /* Status Register */
#define S0SPDR          SPI0->dr        /* Data Register */
#define S0SPCCR         SPI0->ccr       /* Clock Counter Register */
#define S0SPINT         SPI0->flag      /* Interrupt Flag Register */

///////////////////////////////////////////////////////////////////////////////
// Serial Peripheral Interface 1 (SPI1)
#define SPI1            ((spiRegs_t *)0xE0030000)

// SPI1 Registers
#define S1SPCR          SPI1->cr        /* Control Register */
#define S1SPSR          SPI1->sr        /* Status Register */
#define S1SPDR          SPI1->dr        /* Data Register */
#define S1SPCCR         SPI1->ccr       /* Clock Counter Register */
#define S1SPINT         SPI1->flag      /* Interrupt Flag Register */

///////////////////////////////////////////////////////////////////////////////
// Real Time Clock
#define RTC             ((rtcRegs_t *)0xE0024000)

// RTC Registers
#define RTCILR          RTC->ilr        /* Interrupt Location Register */
#define RTCCTC          RTC->ctc        /* Clock Tick Counter */
#define RTCCCR          RTC->ccr        /* Clock Control Register */
#define RTCCIIR         RTC->ciir       /* Counter Increment Interrupt Register */
#define RTCAMR          RTC->amr        /* Alarm Mask Register */
#define RTCCTIME0       RTC->ctime0     /* Consolidated Time Register 0 */
#define RTCCTIME1       RTC->ctime1     /* Consolidated Time Register 1 */
#define RTCCTIME2       RTC->ctime2     /* Consolidated Time Register 2 */
#define RTCSEC          RTC->sec        /* Seconds Register */
#define RTCMIN          RTC->min        /* Minutes Register */
#define RTCHOUR         RTC->hour       /* Hours Register */
#define RTCDOM          RTC->dom        /* Day Of Month Register */
#define RTCDOW          RTC->dow        /* Day Of Week Register */
#define RTCDOY          RTC->doy        /* Day Of Year Register */
#define RTCMONTH        RTC->month      /* Months Register */
#define RTCYEAR         RTC->year       /* Years Register */
#define RTCALSEC        RTC->alsec      /* Alarm Seconds Register */
#define RTCALMIN        RTC->almin      /* Alarm Minutes Register */
#define RTCALHOUR       RTC->alhour     /* Alarm Hours Register */
#define RTCALDOM        RTC->aldom      /* Alarm Day Of Month Register */
#define RTCALDOW        RTC->aldow      /* Alarm Day Of Week Register */
#define RTCALDOY        RTC->aldoy      /* Alarm Day Of Year Register */
#define RTCALMON        RTC->almon      /* Alarm Months Register */
#define RTCALYEAR       RTC->alyear     /* Alarm Years Register */
#define RTCPREINT       RTC->preint     /* Prescale Value Register (integer) */
#define RTCPREFRAC      RTC->prefrac    /* Prescale Value Register (fraction) */

///////////////////////////////////////////////////////////////////////////////
// General Purpose Input/Output
#define GPIO            ((gpioRegs_t *)0xE0028000)

// GPIO Registers
#define IO0PIN          GPIO->in0       /* P0 Pin Value Register */
#define IO0SET          GPIO->set0      /* P0 Pin Output Set Register */
#define IO0DIR          GPIO->dir0      /* P0 Pin Direction Register */
#define IO0CLR          GPIO->clr0      /* P0 Pin Output Clear Register */
#define IO1PIN          GPIO->in1       /* P1 Pin Value Register */
#define IO1SET          GPIO->set1      /* P1 Pin Output Set Register */
#define IO1DIR          GPIO->dir1      /* P1 Pin Direction Register */
#define IO1CLR          GPIO->clr1      /* P1 Pin Output Clear Register */

///////////////////////////////////////////////////////////////////////////////
// Pin Connect Block
#define PINSEL          ((pinRegs_t *)0xE002C000)

// Pin Connect Block Registers
#define PINSEL0         PINSEL->sel0    /* Pin Function Select Register 0 */
#define PINSEL1         PINSEL->sel1    /* Pin Function Select Register 1 */
#define PINSEL2         PINSEL->sel2    /* Pin Function Select Register 2 */

///////////////////////////////////////////////////////////////////////////////
// A/D Converter
#define ADC0            ((adcRegs_t *)0xE0034000)

// A/D0 Converter Registers
#define AD0CR            ADC0->cr       /* Control Register */
#define AD0DR            ADC0->dr       /* Data Register */
#define ADGSR            ADC0->gsr      /* ADC global start resister */

#define ADC1            ((adcRegs_t *)0xE0060000)

// A/D1 Converter Registers
#define AD1CR            ADC1->cr       /* Control Register */
#define AD1DR            ADC1->dr       /* Data Register */

///////////////////////////////////////////////////////////////////////////////
// System Contol Block
#define SCB             ((scbRegs_t *)0xE01FC000)

// Memory Accelerator Module Registers (MAM)
#define MAMCR           SCB->mam.cr     /* Control Register */
#define MAMTIM          SCB->mam.tim    /* Timing Control Register */

// Memory Mapping Control Register
#define MEMMAP          SCB->memmap

// Phase Locked Loop Registers (PLL)
#define PLLCON          SCB->pll.con    /* Control Register */
#define PLLCFG          SCB->pll.cfg    /* Configuration Register */
#define PLLSTAT         SCB->pll.stat   /* Status Register */
#define PLLFEED         SCB->pll.feed   /* Feed Register */

// Power Control Registers
#define PCON            SCB->p.con      /* Control Register */
#define PCONP           SCB->p.conp     /* Peripherals Register */

// VPB Divider Register
#define VPBDIV          SCB->vpbdiv

// External Interrupt Registers
#define EXTINT          SCB->ext.flag   /* Flag Register */
#define EXTWAKE         SCB->ext.wake   /* Wakeup Register */
#define EXTMODE         SCB->ext.mode   /* Mode Register */
#define EXTPOLAR        SCB->ext.polar  /* Polarity Register */

///////////////////////////////////////////////////////////////////////////////
// Vectored Interrupt Controller
#define VIC             ((vicRegs_t *)0xFFFFF000)

// Vectored Interrupt Controller Registers
#define VICIRQStatus    VIC->irqStatus  /* IRQ Status Register */
#define VICFIQStatus    VIC->fiqStatus  /* FIQ Status Register */
#define VICRawIntr      VIC->rawIntr    /* Raw Interrupt Status Register */
#define VICIntSelect    VIC->intSelect  /* Interrupt Select Register */
#define VICIntEnable    VIC->intEnable  /* Interrupt Enable Register */
#define VICIntEnClear   VIC->intEnClear /* Interrupt Enable Clear Register */
#define VICSoftInt      VIC->softInt    /* Software Interrupt Register */
#define VICSoftIntClear VIC->softIntClear /* Software Interrupt Clear Register */
#define VICProtection   VIC->protection /* Protection Enable Register */
#define VICVectAddr     VIC->vectAddr   /* Vector Address Register */
#define VICDefVectAddr  VIC->defVectAddr /* Default Vector Address Register */
#define VICVectAddr0    VIC->vectAddr0  /* Vector Address 0 Register */
#define VICVectAddr1    VIC->vectAddr1  /* Vector Address 1 Register */
#define VICVectAddr2    VIC->vectAddr2  /* Vector Address 2 Register */
#define VICVectAddr3    VIC->vectAddr3  /* Vector Address 3 Register */
#define VICVectAddr4    VIC->vectAddr4  /* Vector Address 4 Register */
#define VICVectAddr5    VIC->vectAddr5  /* Vector Address 5 Register */
#define VICVectAddr6    VIC->vectAddr6  /* Vector Address 6 Register */
#define VICVectAddr7    VIC->vectAddr7  /* Vector Address 7 Register */
#define VICVectAddr8    VIC->vectAddr8  /* Vector Address 8 Register */
#define VICVectAddr9    VIC->vectAddr9  /* Vector Address 9 Register */
#define VICVectAddr10   VIC->vectAddr10 /* Vector Address 10 Register */
#define VICVectAddr11   VIC->vectAddr11 /* Vector Address 11 Register */
#define VICVectAddr12   VIC->vectAddr12 /* Vector Address 12 Register */
#define VICVectAddr13   VIC->vectAddr13 /* Vector Address 13 Register */
#define VICVectAddr14   VIC->vectAddr14 /* Vector Address 14 Register */
#define VICVectAddr15   VIC->vectAddr15 /* Vector Address 15 Register */
#define VICVectCntl0    VIC->vectCntl0  /* Vector Control 0 Register */
#define VICVectCntl1    VIC->vectCntl1  /* Vector Control 1 Register */
#define VICVectCntl2    VIC->vectCntl2  /* Vector Control 2 Register */
#define VICVectCntl3    VIC->vectCntl3  /* Vector Control 3 Register */
#define VICVectCntl4    VIC->vectCntl4  /* Vector Control 4 Register */
#define VICVectCntl5    VIC->vectCntl5  /* Vector Control 5 Register */
#define VICVectCntl6    VIC->vectCntl6  /* Vector Control 6 Register */
#define VICVectCntl7    VIC->vectCntl7  /* Vector Control 7 Register */
#define VICVectCntl8    VIC->vectCntl8  /* Vector Control 8 Register */
#define VICVectCntl9    VIC->vectCntl9  /* Vector Control 9 Register */
#define VICVectCntl10   VIC->vectCntl10 /* Vector Control 10 Register */
#define VICVectCntl11   VIC->vectCntl11 /* Vector Control 11 Register */
#define VICVectCntl12   VIC->vectCntl12 /* Vector Control 12 Register */
#define VICVectCntl13   VIC->vectCntl13 /* Vector Control 13 Register */
#define VICVectCntl14   VIC->vectCntl14 /* Vector Control 14 Register */
#define VICVectCntl15   VIC->vectCntl15 /* Vector Control 15 Register */

#endif
