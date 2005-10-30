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
#ifndef INC_LPC_ADC_H
#define INC_LPC_ADC_H

// A/D Converter Registers
typedef struct
{
  REG32 cr;                             // Control Register
  REG32 dr;                             // Data Register
  REG32 gsr;                            // Global Start Register
} adcRegs_t;

#endif
