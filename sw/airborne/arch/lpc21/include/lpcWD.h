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
#ifndef INC_LPC_WD_H
#define INC_LPC_WD_H

// Watchdog Registers
typedef struct
{
  REG_8 mod;                            // Watchdog Mode Register
  REG_8 _pad0[3];
  REG32 tc;                             // Watchdog Time Constant Register
  REG_8 feed;                           // Watchdog Feed Register
  REG32 tv;                             // Watchdog Time Value Register
} wdRegs_t;

#endif
