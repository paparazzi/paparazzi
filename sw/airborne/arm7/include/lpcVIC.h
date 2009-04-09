/******************************************************************************
 *
 * $RCSfile$
 * $Revision$
 *
 * Header file for Philips LPC ARM Processors.
 * Copyright 2004 R O SoftWare
 *
 * No guarantees, warrantees, or promises, implied or otherwise.
 * May be used for hobby or commercial purposes provided copyright
 * notice remains intact.
 *
 *****************************************************************************/
#ifndef INC_LPC_VIC_H
#define INC_LPC_VIC_H

// Vectored Interrupt Controller Registers (VIC)
typedef struct
{
  REG32 irqStatus;                      // IRQ Status Register
  REG32 fiqStatus;                      // FIQ Status Register
  REG32 rawIntr;                        // Raw Interrupt Status Register
  REG32 intSelect;                      // Interrupt Select Register
  REG32 intEnable;                      // Interrupt Enable Register
  REG32 intEnClear;                     // Interrupt Enable Clear Register
  REG32 softInt;                        // Software Interrupt Register
  REG32 softIntClear;                   // Software Interrupt Clear Register
  REG32 protection;                     // Protection Enable Register
  REG32 _pad0[3];
  REG32 vectAddr;                       // Vector Address Register
  REG32 defVectAddr;                    // Default Vector Address Register
  REG32 _pad1[50];
  REG32 vectAddr0;                      // Vector Address 0 Register
  REG32 vectAddr1;                      // Vector Address 1 Register
  REG32 vectAddr2;                      // Vector Address 2 Register
  REG32 vectAddr3;                      // Vector Address 3 Register
  REG32 vectAddr4;                      // Vector Address 4 Register
  REG32 vectAddr5;                      // Vector Address 5 Register
  REG32 vectAddr6;                      // Vector Address 6 Register
  REG32 vectAddr7;                      // Vector Address 7 Register
  REG32 vectAddr8;                      // Vector Address 8 Register
  REG32 vectAddr9;                      // Vector Address 9 Register
  REG32 vectAddr10;                     // Vector Address 10 Register
  REG32 vectAddr11;                     // Vector Address 11 Register
  REG32 vectAddr12;                     // Vector Address 12 Register
  REG32 vectAddr13;                     // Vector Address 13 Register
  REG32 vectAddr14;                     // Vector Address 14 Register
  REG32 vectAddr15;                     // Vector Address 15 Register
  REG32 _pad2[48];
  REG32 vectCntl0;                      // Vector Control 0 Register
  REG32 vectCntl1;                      // Vector Control 1 Register
  REG32 vectCntl2;                      // Vector Control 2 Register
  REG32 vectCntl3;                      // Vector Control 3 Register
  REG32 vectCntl4;                      // Vector Control 4 Register
  REG32 vectCntl5;                      // Vector Control 5 Register
  REG32 vectCntl6;                      // Vector Control 6 Register
  REG32 vectCntl7;                      // Vector Control 7 Register
  REG32 vectCntl8;                      // Vector Control 8 Register
  REG32 vectCntl9;                      // Vector Control 9 Register
  REG32 vectCntl10;                     // Vector Control 10 Register
  REG32 vectCntl11;                     // Vector Control 11 Register
  REG32 vectCntl12;                     // Vector Control 12 Register
  REG32 vectCntl13;                     // Vector Control 13 Register
  REG32 vectCntl14;                     // Vector Control 14 Register
  REG32 vectCntl15;                     // Vector Control 15 Register
} vicRegs_t;

// VIC Channel Assignments
#define VIC_WDT         0
#define VIC_ARMCore0    2
#define VIC_ARMCore1    3
#define VIC_TIMER0      4
#define VIC_TIMER1      5
#define VIC_UART0       6
#define VIC_UART1       7
#define VIC_PWM         8
#define VIC_PWM0        8
#define VIC_I2C0        9
#define VIC_SPI         10
#define VIC_SPI0        10
#define VIC_SPI1        11
#define VIC_PLL         12
#define VIC_RTC         13
#define VIC_EINT0       14
#define VIC_EINT1       15
#define VIC_EINT2       16
#define VIC_EINT3       17
#define VIC_AD0         18
#define VIC_I2C1        19
#define VIC_BOD         20
#define VIC_AD1         21
#define VIC_USB         22

#define VIC_CAN         19
#define VIC_CAN1_TX     20
#define VIC_CAN2_TX     21
#define VIC_CAN1_RX     26
#define VIC_CAN2_RX     27


// Vector Control Register bit definitions
#define VIC_ENABLE      (1 << 5)

// Convert Channel Number to Bit Value
#define VIC_BIT(chan)   (1 << (chan))

#endif

