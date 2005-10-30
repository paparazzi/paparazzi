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
#ifndef INC_LPC_PIN_H
#define INC_LPC_PIN_H

// Pin Connect Block Registers
typedef struct
{
  REG32 sel0;                           // Pin Function Select Register 0
  REG32 sel1;                           // Pin Function Select Register 1
  REG32 _pad[3];
  REG32 sel2;                           // Pin Function Select Register 2
} pinRegs_t;

#endif
