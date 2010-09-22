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
#ifndef INC_LPC_GPIO_H
#define INC_LPC_GPIO_H

// General Purpose Input/Output Registers (GPIO)
typedef struct
{
  REG32 in0;                            // P0 Pin Value Register
  REG32 set0;                           // P0 Pin Output Set Register
  REG32 dir0;                           // P0 Pin Direction Register
  REG32 clr0;                           // P0 Pin Output Clear Register
  REG32 in1;                            // P1 Pin Value Register
  REG32 set1;                           // P1 Pin Output Set Register
  REG32 dir1;                           // P1 Pin Direction Register
  REG32 clr1;                           // P1 Pin Output Clear Register
  REG32 in2;                            // P2 Pin Value Register
  REG32 set2;                           // P2 Pin Output Set Register
  REG32 dir2;                           // P2 Pin Direction Register
  REG32 clr2;                           // P2 Pin Output Clear Register
  REG32 in3;                            // P3 Pin Value Register
  REG32 set3;                           // P3 Pin Output Set Register
  REG32 dir3;                           // P3 Pin Direction Register
  REG32 clr3;                           // P3 Pin Output Clear Register
} gpioRegs_t;

#endif
