/*
 * Copyright (C) 2011 The Paparazzi Team
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
 */

/*---------------------------------------------------------------------------
  Copyright (c)  Ryan Mechatronics 2008.  All Rights Reserved.

  File: *.c

  Description: CHIMU Protocol Parser


  Public Functions:
  CHIMU_Init           Create component instance
  CHIMU_Done           Free component instance
  CHIMU_Parse          Parse the RX byte stream message

  Applicable Documents:
  CHIMU parsing documentation


  ---------------------------------------------------------------------------*/

#include "paparazzi.h"

//---[Defines]------------------------------------------------------
#ifndef CHIMU_DEFINED_H
#define CHIMU_DEFINED_H

#define CHIMU_STX       0xae
#define CHIMU_BROADCAST   0xaa

// Message ID's that go TO the CHIMU
#define MSG00_PING    0x00
#define MSG01_BIAS    0x01
#define MSG02_DACMODE   0x02
#define MSG03_CALACC    0x03
#define MSG04_CALMAG    0x04
#define MSG05_CALRATE   0x05
#define MSG06_CONFIGCLR   0x06
#define MSG07_CONFIGSET   0x07
#define MSG08_SAVEGYROBIAS  0x08
#define MSG09_ESTIMATOR   0x09
#define MSG0A_SFCHECK   0x0A
#define MSG0B_CENTRIP   0x0B
#define MSG0C_INITGYROS   0x0C
#define MSG0D_DEVICEID    0x0D
#define MSG0E_REFVECTOR   0x0E
#define MSG0F_RESET   0x0F
#define MSG10_UARTSETTINGS  0x10
#define MSG11_SERIALNUMBER  0x11

// Output message identifiers from the CHIMU unit
#define CHIMU_Msg_0_Ping      0
#define CHIMU_Msg_1_IMU_Raw                     1
#define CHIMU_Msg_2_IMU_FP      2
#define CHIMU_Msg_3_IMU_Attitude                3
#define CHIMU_Msg_4_BiasSF      4
#define CHIMU_Msg_5_BIT                         5
#define CHIMU_Msg_6_MagCal      6
#define CHIMU_Msg_7_GyroBias                    7
#define CHIMU_Msg_8_TempCal                     8
#define CHIMU_Msg_9_DAC_Offsets                 9
#define CHIMU_Msg_10_Res      10
#define CHIMU_Msg_11_Res      11
#define CHIMU_Msg_12_Res      12
#define CHIMU_Msg_13_Res      13
#define CHIMU_Msg_14_RefVector                  14
#define CHIMU_Msg_15_SFCheck                    15


/***************************************************************************
 * Endianness Swapping Functions
 */

#ifdef CHIMU_BIG_ENDIAN

static inline float FloatSwap(float f)
{
  union {
    float f;
    unsigned char b[4];
  } dat1, dat2;

  dat1.f = f;
  dat2.b[0] = dat1.b[3];
  dat2.b[1] = dat1.b[2];
  dat2.b[2] = dat1.b[1];
  dat2.b[3] = dat1.b[0];
  return dat2.f;
}

#else

#define FloatSwap(X) (X)

#endif


typedef struct {
  float phi;
  float theta;
  float psi;
} CHIMU_Euler;

typedef struct {
  float x;
  float y;
  float z;
} CHIMU_Vector;

typedef struct {
  float s;
  CHIMU_Vector v;
} CHIMU_Quaternion;

typedef struct {
  CHIMU_Euler euler;
  CHIMU_Quaternion q;
} CHIMU_attitude_data;

#ifndef FALSE
#define FALSE (1==0)
#endif
#ifndef TRUE
#define TRUE (1==1)
#endif

typedef struct {
  float cputemp;
  float acc[3];
  float rate[3];
  float mag[3];
  float spare1;
} CHIMU_sensor_data;

#define CHIMU_RX_BUFFERSIZE 128

typedef struct {
  unsigned char m_State;      // Current state protocol parser is in
  unsigned char   m_Checksum;     // Calculated CHIMU sentence checksum
  unsigned char   m_ReceivedChecksum;   // Received CHIMU sentence checksum (if exists)
  unsigned char   m_Index;      // Index used for command and data
  unsigned char   m_PayloadIndex;
  unsigned char   m_MsgID;
  unsigned char   m_MsgLen;
  unsigned char   m_TempDeviceID;
  unsigned char   m_DeviceID;
  unsigned char   m_Payload[CHIMU_RX_BUFFERSIZE];        // CHIMU data
  unsigned char   m_FullMessage[CHIMU_RX_BUFFERSIZE]; // CHIMU data
  CHIMU_attitude_data m_attitude;
  CHIMU_attitude_data m_attrates;
  CHIMU_sensor_data   m_sensor;

  // Ping data
  uint8_t gCHIMU_SW_Exclaim;
  uint8_t gCHIMU_SW_Major;
  uint8_t gCHIMU_SW_Minor;
  uint16_t gCHIMU_SW_SerialNumber;

  // Config
  uint8_t gCalStatus;
  uint8_t gCHIMU_BIT;
  uint8_t gConfigInfo;

} CHIMU_PARSER_DATA;

/*---------------------------------------------------------------------------
  Name: CHIMU_Init
  ---------------------------------------------------------------------------*/
void CHIMU_Init(CHIMU_PARSER_DATA   *pstData);

/*---------------------------------------------------------------------------
  Name: CHIMU_Parse
  Abstract: Parse message input test mode, returns TRUE if new data.
  ---------------------------------------------------------------------------*/
unsigned char CHIMU_Parse(unsigned char btData, unsigned char bInputType, CHIMU_PARSER_DATA *pstData);

unsigned char CHIMU_ProcessMessage(unsigned char *pMsgID, unsigned char *pPayloadData, CHIMU_PARSER_DATA  *pstData);

void CHIMU_Checksum(unsigned char *data, unsigned char buflen);

#endif // CHIMU_DEFINED
