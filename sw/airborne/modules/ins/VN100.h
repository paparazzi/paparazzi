/*
 * Copyright (C) 2010 ENAC
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * \brief Library for the VectorNav VN100 AHRS
 * based on VN_FWLIB from vector nav
 */

#ifndef VN100_H
#define VN100_H

#include "std.h"

/* VN-100 Registers */
#define VN100_REG_MODEL     1
#define VN100_REG_HWREV     2
#define VN100_REG_SN        3
#define VN100_REG_FWVER     4
#define VN100_REG_SBAUD     5
#define VN100_REG_ADOR      6
#define VN100_REG_ADOF      7
#define VN100_REG_YPR       8
#define VN100_REG_QTN       9
#define VN100_REG_QTM       10
#define VN100_REG_QTA       11
#define VN100_REG_QTR       12
#define VN100_REG_QMA       13
#define VN100_REG_QAR       14
#define VN100_REG_QMR       15
#define VN100_REG_DCM       16
#define VN100_REG_MAG       17
#define VN100_REG_ACC       18
#define VN100_REG_GYR       19
#define VN100_REG_MAR       20
#define VN100_REG_REF       21
#define VN100_REG_SIG       22
#define VN100_REG_HSI       23
#define VN100_REG_ATP       24
#define VN100_REG_ACT       25
#define VN100_REG_RFR       26
#define VN100_REG_YMR       27
#define VN100_REG_ACG       28

/* Data Size */
#define VN100_REG_MODEL_SIZE    12
#define VN100_REG_HWREV_SIZE    4
#define VN100_REG_SN_SIZE       12
#define VN100_REG_FWVER_SIZE    4
#define VN100_REG_SBAUD_SIZE    4
#define VN100_REG_ADOR_SIZE     4
#define VN100_REG_ADOF_SIZE     4
#define VN100_REG_YPR_SIZE      (3*4)
#define VN100_REG_QTN_SIZE      (4*4)
#define VN100_REG_QTM_SIZE      (7*4)
#define VN100_REG_QTA_SIZE      (7*4)
#define VN100_REG_QTR_SIZE      (7*4)
#define VN100_REG_QMA_SIZE      (10*4)
#define VN100_REG_QAR_SIZE      (10*4)
#define VN100_REG_QMR_SIZE      (13*4)
#define VN100_REG_DCM_SIZE      (9*4)
#define VN100_REG_MAG_SIZE      (3*4)
#define VN100_REG_ACC_SIZE      (3*4)
#define VN100_REG_GYR_SIZE      (3*4)
#define VN100_REG_MAR_SIZE      (9*4)
#define VN100_REG_REF_SIZE      (6*4)
#define VN100_REG_SIG_SIZE      (10*4)
#define VN100_REG_HSI_SIZE      (12*4)
#define VN100_REG_ATP_SIZE      (4*4)
#define VN100_REG_ACT_SIZE      (12*4)
#define VN100_REG_RFR_SIZE      (9*4)
#define VN100_REG_YMR_SIZE      (12*4)
#define VN100_REG_ACG_SIZE      4

#define VN100_DataSizeOfReg(_r) (_r##_SIZE)

/* Command IDs */
#define VN100_CmdID_ReadRegister             0x01
#define VN100_CmdID_WriteRegister            0x02
#define VN100_CmdID_WriteSettings            0x03
#define VN100_CmdID_RestoreFactorySettings   0x04
#define VN100_CmdID_Tare                     0x05
#define VN100_CmdID_Reset                    0x06

/* System Error */
#define VN100_Error_None                     0
#define VN100_Error_HardFaultException       1
#define VN100_Error_InputBufferOverflow      2
#define VN100_Error_InvalidChecksum          3
#define VN100_Error_InvalidCommand           4
#define VN100_Error_NotEnoughParameters      5
#define VN100_Error_TooManyParameters        6
#define VN100_Error_InvalidParameter         7
#define VN100_Error_InvalidRegister          8
#define VN100_Error_UnauthorizedAccess       9
#define VN100_Error_WatchdogReset            10
#define VN100_Error_OutputBufferOverflow     11
#define VN100_Error_InsufficientBandwidth    12

/* Asynchronous Data Output Register */
#define VN100_ADOR_OFF  0
#define VN100_ADOR_YPR  1
#define VN100_ADOR_QTN  2
#define VN100_ADOR_QTM  3
#define VN100_ADOR_QTA  4
#define VN100_ADOR_QTR  5
#define VN100_ADOR_QMA  6
#define VN100_ADOR_QAR  7
#define VN100_ADOR_QMR  8
#define VN100_ADOR_DCM  9
#define VN100_ADOR_MAG  10
#define VN100_ADOR_ACC  11
#define VN100_ADOR_GYR  12
#define VN100_ADOR_MAR  13
#define VN100_ADOR_YMR  14
#define VN100_ADOR_RAB  251
#define VN100_ADOR_RAW  252
#define VN100_ADOR_CMV  253
#define VN100_ADOR_STV  254
#define VN100_ADOR_COV  255

/* Asynchronous Data Ouput Rate Register */
#define VN100_ADOF_1HZ    1
#define VN100_ADOF_2HZ    2
#define VN100_ADOF_4HZ    4
#define VN100_ADOF_5HZ    5
#define VN100_ADOF_10HZ   10
#define VN100_ADOF_20HZ   20
#define VN100_ADOF_25HZ   25
#define VN100_ADOF_40HZ   40
#define VN100_ADOF_50HZ   50
#define VN100_ADOF_100HZ  100
#define VN100_ADOF_200HZ  200

/* Serial Baud Rate Register */
#define VN100_Baud_9600    9600
#define VN100_Baud_19200   19200
#define VN100_Baud_38400   38400
#define VN100_Baud_57600   57600
#define VN100_Baud_115200  115200
#define VN100_Baud_128000  128000
#define VN100_Baud_230400  230400
#define VN100_Baud_460800  460800
#define VN100_Baud_921600  921600

/* Accelerometer Gain Type */
#define VN100_AccGain_2G  0
#define VN100_AccGain_6G  1

/* 32-bit Parameter Type */
typedef union {
  uint32_t UInt;
  float    Float;
} VN100_Param;

/* SPI Buffer size */
#define VN100_SPI_BUFFER_SIZE  48

/* SPI Request Packet */
typedef struct {
  uint8_t CmdID;
  uint8_t RegID;
  uint8_t ZeroByte1;
  uint8_t ZeroByte2;
  VN100_Param Data[VN100_SPI_BUFFER_SIZE];
} VN100_Req_Packet;

/* SPI Response Packet */
typedef struct {
  uint8_t ZeroByte;
  uint8_t CmdID;
  uint8_t RegID;
  uint8_t ErrID;
  VN100_Param Data[VN100_SPI_BUFFER_SIZE];
} VN100_Res_Packet;

#define VN100_Packet_SetBaud(_b) { VN100_CmdID_WriteRegister, VN100_REG_SBAUD, 0, 0, { _b } }
#define VN100_Packet_SetADOR(_r) { VN100_CmdID_WriteRegister, VN100_REG_ADOR, 0, 0, { _r } }
#define VN100_Packet_SetaDOF(_f) { VN100_CmdID_WriteRegister, VN100_REG_ADOF, 0, 0, { _f } }

#define VN100_BytesOfWord(_w) { (uint8_t)(_w & 0xFF), (uint8_t)((_w & (0xFF<<8))>>8), (uint8_t)((_w & (0xFF<<16))>>16), (uint8_t)((_w & (0xFF<<24))>>24) }
#define VN100_WordOfBytes(_b) (((uint32_t)(_b[3])<<24)|((uint32_t)(_b[2])<<16)|((uint16_t)(_b[1])<<8)|(uint16_t)(_b[0]))

#endif
