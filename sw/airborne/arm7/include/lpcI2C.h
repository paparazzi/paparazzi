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
#ifndef INC_LPC_I2C_H
#define INC_LPC_I2C_H

// I2C Interface Registers
typedef struct
{
  REG_8 conset;                         // Control Set Register
  REG_8 _pad0[3];
  REG_8 stat;                           // Status Register
  REG_8 _pad1[3];
  REG_8 dat;                            // Data Register
  REG_8 _pad2[3];
  REG_8 adr;                            // Slave Address Register
  REG_8 _pad3[3];
  REG16 sclh;                           // SCL Duty Cycle Register (high half word)
  REG16 _pad4;
  REG16 scll;                           // SCL Duty Cycle Register (low half word)
  REG16 _pad5;
  REG_8 conclr;                         // Control Clear Register
  REG_8 _pad6[3];
} i2cRegs_t;

#endif
