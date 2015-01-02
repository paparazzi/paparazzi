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
  CHIMU_Init        Create component instance
  CHIMU_Parse       Parse the RX byte stream message

  Applicable Documents:
  CHIMU User Manual

  Adapted to paparazzi by C. De Wagter

  ---------------------------------------------------------------------------*/

#include "imu_chimu.h"
#include "string.h"
#include "math.h"


/***************************************************************************
 * Cyclic Redundancy Checksum
 */

static unsigned long UpdateCRC(unsigned long CRC_acc, void *data, unsigned long data_len)
{
  unsigned long i; // loop counter
#define POLY 0xEDB88320 // bit-reversed version of the poly 0x04C11DB7
  // Create the CRC "dividend" for polynomial arithmetic (binary arithmetic
  // with no carries)

  unsigned char *CRC_input = (unsigned char *)data;
  for (unsigned long j = data_len; j; --j) {

    CRC_acc = CRC_acc ^ *CRC_input++;
    // "Divide" the poly into the dividend using CRC XOR subtraction
    // CRC_acc holds the "remainder" of each divide
    //
    // Only complete this division for 8 bits since input is 1 byte
    for (i = 0; i < 8; i++)  {
      // Check if the MSB is set (if MSB is 1, then the POLY can "divide"
      // into the "dividend")
      if ((CRC_acc & 0x00000001) == 0x00000001) {
        // if so, shift the CRC value, and XOR "subtract" the poly
        CRC_acc = CRC_acc >> 1;
        CRC_acc ^= POLY;
      } else {
        // if not, just shift the CRC value
        CRC_acc = CRC_acc >> 1;
      }
    }
  }
  // Return the final remainder (CRC value)
  return CRC_acc;
}

void CHIMU_Checksum(unsigned char *data, unsigned char buflen)
{
  data[buflen - 1] = (unsigned char)(UpdateCRC(0xFFFFFFFF , data , (unsigned long)(buflen - 1)) & 0xff) ;
}


/***************************************************************************
 *  CHIMU Protocol Definition
 */

// Lowlevel Protocol Decoding
#define CHIMU_STATE_MACHINE_START       0
#define CHIMU_STATE_MACHINE_HEADER2     1
#define CHIMU_STATE_MACHINE_LEN   2
#define CHIMU_STATE_MACHINE_DEVICE  3
#define CHIMU_STATE_MACHINE_ID    4
#define CHIMU_STATE_MACHINE_PAYLOAD 5
#define CHIMU_STATE_MACHINE_XSUM  6

// Communication Definitions
#define CHIMU_COM_ID_HIGH 0x1F  //Must set this to the max ID expected above

/*---------------------------------------------------------------------------
  Name: CHIMU_Init

  ---------------------------------------------------------------------------*/
void CHIMU_Init(CHIMU_PARSER_DATA   *pstData)
{
  unsigned char i;
  pstData->m_State = CHIMU_STATE_MACHINE_START;
  pstData->m_Checksum = 0x00;
  pstData->m_ReceivedChecksum = 0x00;
  pstData->m_Index = 0;
  pstData->m_PayloadIndex = 0;

  //Sensor data holder
  pstData->m_sensor.cputemp = 0.0;
  for (i = 0; i < 3; i++) {
    pstData->m_sensor.acc[i] = 0.0;
    pstData->m_sensor.rate[i] = 0.0;
    pstData->m_sensor.mag[i] = 0.0;
  }
  pstData->m_sensor.spare1 = 0.0;
  //Attitude data
  pstData->m_attitude.euler.phi = 0.0;
  pstData->m_attitude.euler.theta = 0.0;
  pstData->m_attitude.euler.psi = 0.0;
  //Attitude rate data
  pstData->m_attrates.euler.phi = 0.0;
  pstData->m_attrates.euler.theta = 0.0;
  pstData->m_attrates.euler.psi = 0.0;

  for (i = 0; i < CHIMU_RX_BUFFERSIZE; i++) {
    pstData->m_Payload[i] = 0x00;
    pstData->m_FullMessage[i] = 0x00;
  }
  pstData->m_MsgLen = 0;
  pstData->m_MsgID = 0;
  pstData->m_TempDeviceID = 0;
  pstData->m_DeviceID = 0x01; //look at this later
}

/*---------------------------------------------------------------------------
  Name: CHIMU_Parse
  Abstract: Parse message, returns TRUE if new data.
  ---------------------------------------------------------------------------*/

unsigned char CHIMU_Parse(
  unsigned char btData,           /* input byte stream buffer */
  unsigned char bInputType __attribute__((
        unused)),       /* for future use if special builds of CHIMU data are performed */
  CHIMU_PARSER_DATA   *pstData)   /* resulting data           */
{

  char           bUpdate = FALSE;

  switch (pstData->m_State) {
    case CHIMU_STATE_MACHINE_START:  // Waiting for start character 0xAE
      if (btData == 0xAE) {
        pstData->m_State = CHIMU_STATE_MACHINE_HEADER2;
        pstData->m_Index = 0;
        pstData->m_FullMessage[pstData->m_Index++] = btData;
      } else {
        ;;
      }
      bUpdate = FALSE;
      break;
    case CHIMU_STATE_MACHINE_HEADER2:  // Waiting for second header character 0xAE
      if (btData == 0xAE) {
        pstData->m_State = CHIMU_STATE_MACHINE_LEN;
        pstData->m_FullMessage[pstData->m_Index++] = btData;
      } else {
        pstData->m_State = CHIMU_STATE_MACHINE_START;
      } //Fail to see header.  Restart.
      break;
    case CHIMU_STATE_MACHINE_LEN:  // Get chars to read
      if (btData <= CHIMU_RX_BUFFERSIZE) {
        pstData->m_MsgLen = btData  ; // It might be invalid, but we do a check on buffer size
        pstData->m_FullMessage[pstData->m_Index++] = btData;
        pstData->m_State = CHIMU_STATE_MACHINE_DEVICE;
      } else {
        pstData->m_State = CHIMU_STATE_MACHINE_START; //Length byte exceeds buffer.  Signal a fail and restart
        //BuiltInTest(BIT_COM_UART_RECEIPTFAIL, BIT_FAIL);
      }
      break;
    case CHIMU_STATE_MACHINE_DEVICE:  // Get device.  If not us, ignore and move on.  Allows common com with Monkey / Chipmunk
      if ((btData == pstData->m_DeviceID) || (btData == 0xAA))  {
        //0xAA is global message
        pstData->m_TempDeviceID = btData;
        pstData->m_FullMessage[pstData->m_Index++] = btData;
        pstData->m_State = CHIMU_STATE_MACHINE_ID;
      } else {
        pstData->m_State = CHIMU_STATE_MACHINE_START;
      } //Fail to see correct device ID.  Restart.
      break;
    case CHIMU_STATE_MACHINE_ID:  // Get ID
      pstData->m_MsgID = btData; // might be invalid, chgeck it out here:
      if (pstData->m_MsgID > CHIMU_COM_ID_HIGH) {
        pstData->m_State = CHIMU_STATE_MACHINE_START;
        //BuiltInTest(BIT_COM_UART_RECEIPTFAIL, BIT_FAIL);
      } else {
        pstData->m_FullMessage[pstData->m_Index++] = btData;
        pstData->m_PayloadIndex = 0;
        pstData->m_State = CHIMU_STATE_MACHINE_PAYLOAD; //Finally....Good to go...
      }
      break;
    case CHIMU_STATE_MACHINE_PAYLOAD:  // Waiting for number of bytes in payload
      pstData->m_Payload[pstData->m_PayloadIndex++] = btData;
      pstData->m_FullMessage[pstData->m_Index++] = btData;
      if ((pstData->m_Index) >= (pstData->m_MsgLen + 5)) {
        //Now we have the payload.  Verify XSUM and then parse it next
        pstData->m_Checksum = (unsigned char)((UpdateCRC(0xFFFFFFFF , pstData->m_FullMessage ,
                                               (unsigned long)(pstData->m_MsgLen) + 5)) & 0xFF);
        pstData->m_State = CHIMU_STATE_MACHINE_XSUM;
      } else {
        return FALSE;
      }
      break;
    case CHIMU_STATE_MACHINE_XSUM:  // Verify
      pstData->m_ReceivedChecksum = btData;
      pstData->m_FullMessage[pstData->m_Index++] = btData;
      if (pstData->m_Checksum != pstData->m_ReceivedChecksum) {
        bUpdate = FALSE;
        //BuiltInTest(BIT_COM_UART_RECEIPTFAIL, BIT_FAIL);
      } else {
        //Xsum passed, go parse it.
        // We have pstData->m_MsgID to parse off of, pstData->m_pstData->m_Payload as the data.
        bUpdate = CHIMU_ProcessMessage(&pstData->m_MsgID, pstData->m_Payload, pstData);
      }
      pstData->m_State = CHIMU_STATE_MACHINE_START;
      break;
    default:
      pstData->m_State = CHIMU_STATE_MACHINE_START;
  } /* End of SWITCH */
  return (bUpdate);
}


///////////////////////////////////////////////////////////////////////////////
// Process CHIMU sentence - Use the CHIMU address (*pCommand) and call the
// appropriate sentence data processor.
///////////////////////////////////////////////////////////////////////////////

static CHIMU_attitude_data GetEulersFromQuat(CHIMU_attitude_data attitude)
{
  CHIMU_attitude_data ps;
  ps = attitude;
  float x, sqw, sqx, sqy, sqz, norm;
  sqw = ps.q.s * ps.q.s;
  sqx = ps.q.v.x * ps.q.v.x;
  sqy = ps.q.v.y * ps.q.v.y;
  sqz = ps.q.v.z * ps.q.v.z;
  norm = sqrt(sqw + sqx + sqy + sqz);
  //Normalize the quat
  ps.q.s = ps.q.s / norm;
  ps.q.v.x = ps.q.v.x / norm;
  ps.q.v.y = ps.q.v.y / norm;
  ps.q.v.z = ps.q.v.z / norm;
  ps.euler.phi = atan2(2.0 * (ps.q.s * ps.q.v.x + ps.q.v.y * ps.q.v.z), (1 - 2 * (sqx + sqy)));
  if (ps.euler.phi < 0) { ps.euler.phi = ps.euler.phi + 2 * M_PI; }
  x = ((2.0 * (ps.q.s * ps.q.v.y - ps.q.v.z * ps.q.v.x)));
  //Below needed in event normalization not done
  if (x > 1.0) { x = 1.0; }
  if (x < -1.0) { x = -1.0; }
  //
  if ((ps.q.v.x * ps.q.v.y + ps.q.v.z * ps.q.s) == 0.5)  {
    ps.euler.theta = 2 * atan2(ps.q.v.x, ps.q.s);
  } else if ((ps.q.v.x * ps.q.v.y + ps.q.v.z * ps.q.s) == -0.5)  {
    ps.euler.theta = -2 * atan2(ps.q.v.x, ps.q.s);
  } else {
    ps.euler.theta = asin(x);
  }
  ps.euler.psi = atan2(2.0 * (ps.q.s * ps.q.v.z + ps.q.v.x * ps.q.v.y), (1 - 2 * (sqy + sqz)));
  if (ps.euler.psi < 0) {
    ps.euler.psi = ps.euler.psi + (2 * M_PI);
  }

  return (ps);

}

static unsigned char BitTest(unsigned char input, unsigned char n)
{
  //Test a bit in n and return TRUE or FALSE
  if (input & (1 << n)) { return TRUE; } else { return FALSE; }
}
unsigned char CHIMU_ProcessMessage(unsigned char *pMsgID __attribute__((unused)), unsigned char *pPayloadData,
                                   CHIMU_PARSER_DATA *pstData)
{
  //Msgs from CHIMU are off limits (i.e.any CHIMU messages sent up the uplink should go to
  //CHIMU).

  //Any CHIMU messages coming from the ground should be ignored, as that byte stream goes up to CHIMU
  // by itself.  However, here we should decode CHIMU messages being received and
  //  a) pass them down to ground
  //  b) grab the data from the CHIMU for our own needs / purposes
  int CHIMU_index = 0;
  float sanity_check = 0.0;

  switch (pstData->m_MsgID) {
    case CHIMU_Msg_0_Ping:
      CHIMU_index = 0;
      pstData->gCHIMU_SW_Exclaim = pPayloadData[CHIMU_index]; CHIMU_index++;
      pstData->gCHIMU_SW_Major = pPayloadData[CHIMU_index]; CHIMU_index++;
      pstData->gCHIMU_SW_Minor = pPayloadData[CHIMU_index]; CHIMU_index++;
      pstData->gCHIMU_SW_SerialNumber = (pPayloadData[CHIMU_index] << 8) & (0x0000FF00); CHIMU_index++;
      pstData->gCHIMU_SW_SerialNumber += pPayloadData[CHIMU_index]; CHIMU_index++;

      return TRUE;
      break;
    case CHIMU_Msg_1_IMU_Raw:
      break;
    case CHIMU_Msg_2_IMU_FP:
      CHIMU_index = 0;
      memmove(&pstData->m_sensor.cputemp, &pPayloadData[CHIMU_index], sizeof(pstData->m_sensor.cputemp));
      CHIMU_index += (sizeof(pstData->m_sensor.cputemp));
      pstData->m_sensor.cputemp = FloatSwap(pstData->m_sensor.cputemp);
      memmove(&pstData->m_sensor.acc[0], &pPayloadData[CHIMU_index], sizeof(pstData->m_sensor.acc));
      CHIMU_index += (sizeof(pstData->m_sensor.acc));
      pstData->m_sensor.acc[0] = FloatSwap(pstData->m_sensor.acc[0]);
      pstData->m_sensor.acc[1] = FloatSwap(pstData->m_sensor.acc[1]);
      pstData->m_sensor.acc[2] = FloatSwap(pstData->m_sensor.acc[2]);
      memmove(&pstData->m_sensor.rate[0], &pPayloadData[CHIMU_index], sizeof(pstData->m_sensor.rate));
      CHIMU_index += (sizeof(pstData->m_sensor.rate));
      pstData->m_sensor.rate[0] = FloatSwap(pstData->m_sensor.rate[0]);
      pstData->m_sensor.rate[1] = FloatSwap(pstData->m_sensor.rate[1]);
      pstData->m_sensor.rate[2] = FloatSwap(pstData->m_sensor.rate[2]);
      memmove(&pstData->m_sensor.mag[0], &pPayloadData[CHIMU_index], sizeof(pstData->m_sensor.mag));
      CHIMU_index += (sizeof(pstData->m_sensor.mag));
      pstData->m_sensor.mag[0] = FloatSwap(pstData->m_sensor.mag[0]);
      pstData->m_sensor.mag[1] = FloatSwap(pstData->m_sensor.mag[1]);
      pstData->m_sensor.mag[2] = FloatSwap(pstData->m_sensor.mag[2]);
      memmove(&pstData->m_sensor.spare1, &pPayloadData[CHIMU_index], sizeof(pstData->m_sensor.spare1));
      CHIMU_index += (sizeof(pstData->m_sensor.spare1));
      pstData->m_sensor.spare1 = FloatSwap(pstData->m_sensor.spare1);
      return TRUE;
      break;
    case CHIMU_Msg_3_IMU_Attitude:
      //Attitude message data from CHIMU
      // Includes attitude and rates only, along with configuration bits
      // All you need for control

      //Led_On(LED_RED);

      CHIMU_index = 0;
      memmove(&pstData->m_attitude.euler, &pPayloadData[CHIMU_index], sizeof(pstData->m_attitude.euler));
      CHIMU_index += sizeof(pstData->m_attitude.euler);
      pstData->m_attitude.euler.phi = FloatSwap(pstData->m_attitude.euler.phi);
      pstData->m_attitude.euler.theta = FloatSwap(pstData->m_attitude.euler.theta);
      pstData->m_attitude.euler.psi = FloatSwap(pstData->m_attitude.euler.psi);
      memmove(&pstData->m_sensor.rate[0], &pPayloadData[CHIMU_index], sizeof(pstData->m_sensor.rate));
      CHIMU_index += (sizeof(pstData->m_sensor.rate));
      pstData->m_sensor.rate[0] = FloatSwap(pstData->m_sensor.rate[0]);
      pstData->m_sensor.rate[1] = FloatSwap(pstData->m_sensor.rate[1]);
      pstData->m_sensor.rate[2] = FloatSwap(pstData->m_sensor.rate[2]);

      memmove(&pstData->m_attitude.q, &pPayloadData[CHIMU_index], sizeof(pstData->m_attitude.q));
      CHIMU_index += sizeof(pstData->m_attitude.q);
      pstData->m_attitude.q.s = FloatSwap(pstData->m_attitude.q.s);
      pstData->m_attitude.q.v.x = FloatSwap(pstData->m_attitude.q.v.x);
      pstData->m_attitude.q.v.y = FloatSwap(pstData->m_attitude.q.v.y);
      pstData->m_attitude.q.v.z = FloatSwap(pstData->m_attitude.q.v.z);

      memmove(&pstData->m_attrates.q, &pPayloadData[CHIMU_index], sizeof(pstData->m_attrates.q));
      CHIMU_index += sizeof(pstData->m_attitude.q);
      pstData->m_attrates.q.s = FloatSwap(pstData->m_attrates.q.s);
      pstData->m_attrates.q.v.x = FloatSwap(pstData->m_attrates.q.v.x);
      pstData->m_attrates.q.v.y = FloatSwap(pstData->m_attrates.q.v.y);
      pstData->m_attrates.q.v.z = FloatSwap(pstData->m_attrates.q.v.z);

      //Now put the rates into the Euler section as well.  User can use pstData->m_attitude and pstData->m_attrates structures for control
      pstData->m_attrates.euler.phi = pstData->m_sensor.rate[0];
      pstData->m_attrates.euler.theta = pstData->m_sensor.rate[1];
      pstData->m_attrates.euler.psi = pstData->m_sensor.rate[2];

      pstData->gCalStatus = pPayloadData[CHIMU_index]; CHIMU_index ++;
      pstData->gCHIMU_BIT = pPayloadData[CHIMU_index]; CHIMU_index ++;
      pstData->gConfigInfo = pPayloadData[CHIMU_index]; CHIMU_index ++;

      // TODO: Read configuration bits

      /*                  bC0_SPI_En = BitTest (gConfigInfo, 0);
                          bC1_HWCentrip_En = BitTest (gConfigInfo, 1);
                          bC2_TempCal_En = BitTest (gConfigInfo, 2);
                          bC3_RateOut_En = BitTest (gConfigInfo, 3);
                          bC4_TBD = BitTest (gConfigInfo, 4);
                          bC5_Quat_Est = BitTest (gConfigInfo, 5);
                          bC6_SWCentrip_En = BitTest (gConfigInfo, 6);
                          bC7_AllowHW_Override = BitTest (gConfigInfo, 7);
      */
      //CHIMU currently (v 1.3) does not compute Eulers if quaternion estimator is selected
      if (BitTest(pstData->gConfigInfo, 5) == TRUE) {
        pstData->m_attitude = GetEulersFromQuat((pstData->m_attitude));
      }

      //NEW:  Checks for bad attitude data (bad SPI maybe?)
      //      Only allow globals to contain updated data if it makes sense
      sanity_check = (pstData->m_attitude.q.s * pstData->m_attitude.q.s);
      sanity_check += (pstData->m_attitude.q.v.x * pstData->m_attitude.q.v.x);
      sanity_check += (pstData->m_attitude.q.v.y * pstData->m_attitude.q.v.y);
      sanity_check += (pstData->m_attitude.q.v.z * pstData->m_attitude.q.v.z);

      //Should be 1.0 (normalized quaternion)
      if ((sanity_check > 0.8) && (sanity_check < 1.2)) {
        //                    gAttitude = pstData->m_attitude;
        //                    gAttRates = pstData->m_attrates;
        //                    gSensor = pstData->m_sensor;
      } else {
        //TODO:  Log BIT that indicates IMU message incoming failed (maybe SPI error?)
      }

      return TRUE;
      break;
    case CHIMU_Msg_4_BiasSF:
    case CHIMU_Msg_5_BIT:
    case CHIMU_Msg_6_MagCal:
    case CHIMU_Msg_7_GyroBias:
    case CHIMU_Msg_8_TempCal:
    case CHIMU_Msg_9_DAC_Offsets:
    case CHIMU_Msg_10_Res:
    case CHIMU_Msg_11_Res:
    case CHIMU_Msg_12_Res:
    case CHIMU_Msg_13_Res:
    case CHIMU_Msg_14_RefVector:
    case CHIMU_Msg_15_SFCheck:
      break;
    default:
      return FALSE;
      break;
  }
  return FALSE;
}
