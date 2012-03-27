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
#ifndef INC_LPC_SPI_H
#define INC_LPC_SPI_H

// Serial Peripheral Interface Registers (SPI)
typedef struct
{
  REG_8 cr;                             // Control Register
  REG_8 _pad0[3];
  REG_8 sr;                             // Status Register
  REG_8 _pad1[3];
  REG_8 dr;                             // Data Register
  REG_8 _pad2[3];
  REG_8 ccr;                            // Clock Counter Register
  REG_8 _pad3[3];
  REG_8 tcr;                            // Test Control Register
  REG_8 _pad4[3];
  REG_8 tsr;                            // Test Status Register
  REG_8 _pad5[3];
  REG_8 tor;                            // Test Observe Register
  REG_8 _pad6[3];
  REG_8 flag;                           // Interrupt Flag Register
  REG_8 _pad7[3];
} spiRegs_t;

// Serial Serial Port Registers (SSP/SPI)
typedef struct
{
  REG16 cr0;                            // Control Register 0
  REG16 _pad0;
  REG_8 cr1;                            // Control Register 1
  REG_8 _pad1[3];
  REG16 dr;                             // Data Register
  REG16 _pad2;
  REG_8 sr;                             // Status Register
  REG_8 _pad3[3];
  REG_8 cpsr;                           // Clock Prescale Register
  REG_8 _pad4[3];
  REG_8 imsc;                           // Interrupt Mask Set and Clear Register
  REG_8 _pad5[3];
  REG_8 ris;                            // Raw Interrupt Status Register
  REG_8 _pad6[3];
  REG_8 mis;                            // Masked Interrupt Status Register
  REG_8 _pad7[3];
  REG_8 icr;                            // Interrupt Clear Register
  REG_8 _pad8[3];
} sspRegs_t;

#endif
