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
#ifndef INC_LPC_EMC_H
#define INC_LPC_EMC_H

// External Memory Controller Registers
typedef struct
{
  REG32 bcfg0;                          // Bank 0 Configuration Register
  REG32 bcfg1;                          // Bank 1 Configuration Register
  REG32 bcfg2;                          // Bank 2 Configuration Register
  REG32 bcfg3;                          // Bank 3 Configuration Register
} emcRegs_t;

#endif
