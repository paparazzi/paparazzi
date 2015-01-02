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
 * \brief driver for the VectorNav VN100
 */

#include "modules/ins/ins_vn100.h"

#include "generated/airframe.h"
#include "mcu_periph/spi.h"
#include "state.h"

// for telemetry report
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"
#include "messages.h"

#ifndef INS_YAW_NEUTRAL_DEFAULT
#define INS_YAW_NEUTRAL_DEFAULT 0.
#endif

// default spi device
#ifndef VN100_SPI_DEV
#define VN100_SPI_DEV spi1
#endif

// default slave number
#ifndef VN100_SLAVE_IDX
#define VN100_SLAVE_IDX 0
#endif

/* neutrals */
float ins_roll_neutral;
float ins_pitch_neutral;
float ins_yaw_neutral;

struct FloatEulers ins_eulers;
struct FloatQuat ins_quat;
struct FloatRates ins_rates;
struct FloatRMat ins_rmat;
struct FloatVect3 ins_accel;
struct FloatVect3 ins_mag;

volatile uint8_t ins_msg_received;

/* last received SPI packet */
VN100_Res_Packet last_received_packet;
/* last send packet */
VN100_Req_Packet last_send_packet;

/* output mode */
uint32_t ins_ador;
uint32_t ins_adof;
uint32_t ins_baud;

uint8_t ins_init_status;

// parsing function
static inline void parse_ins_msg(void);

/* spi transaction */
struct spi_transaction vn100_trans;

/* init vn100 */
void vn100_init(void)
{

  //ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
  //ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
  ins_yaw_neutral = INS_YAW_NEUTRAL_DEFAULT;

  vn100_trans.slave_idx = VN100_SLAVE_IDX;
  vn100_trans.cpol = SPICpolIdleHigh;
  vn100_trans.cpha = SPICphaEdge2;
  vn100_trans.dss = SPIDss8bit;
  vn100_trans.select = SPISelectUnselect;
  vn100_trans.output_buf = (uint8_t *)&last_send_packet;
  vn100_trans.input_buf = (uint8_t *)&last_received_packet;
  vn100_trans.status = SPITransDone;

  ins_ador = VN100_ADOR;
  ins_adof = VN100_ADOF;
  ins_baud = VN100_BAUD;

  ins_init_status = INS_VN100_SET_BAUD;

}

static inline bool_t ins_configure(void)
{
  // nothing to receive during conf
  vn100_trans.input_length = 0;

  switch (ins_init_status) {
    case INS_VN100_SET_BAUD :
      last_send_packet.RegID = VN100_REG_SBAUD;
      vn100_trans.output_length = 4 + VN100_REG_SBAUD_SIZE;
      ins_init_status++;
      break;
    case INS_VN100_SET_ADOR :
      last_send_packet.RegID = VN100_REG_ADOR;
      vn100_trans.output_length = 4 + VN100_REG_ADOR_SIZE;
      ins_init_status++;
      break;
    case INS_VN100_SET_ADOF :
      last_send_packet.RegID = VN100_REG_ADOF;
      vn100_trans.output_length = 4 + VN100_REG_ADOF_SIZE;
      ins_init_status++;
      break;
    case INS_VN100_READY :
      return TRUE;
  }
  last_send_packet.CmdID = VN100_CmdID_WriteRegister;

  spi_submit(&(VN100_SPI_DEV), &vn100_trans);

  return FALSE;
}

void vn100_periodic_task(void)
{

  // only send config or request when last transaction is done
  if (vn100_trans.status != SPITransDone) { return; }

  // send request when configuration is done
  if (ins_configure() == TRUE) {
    // Fill request for QMR
    last_send_packet.CmdID = VN100_CmdID_ReadRegister;
    last_send_packet.RegID = VN100_REG_YMR;
    // Set IO length
    vn100_trans.output_length = 2; // Only 2 ?
    vn100_trans.input_length = 4 + VN100_REG_YMR_SIZE;
    // submit
    spi_submit(&(VN100_SPI_DEV), &vn100_trans);
  }

}

void vn100_event_task(void)
{
  if (vn100_trans.status == SPITransSuccess) {
    parse_ins_msg();
#ifndef INS_VN100_READ_ONLY
    // Update estimator
    // FIXME Use a proper rotation matrix here
    struct FloatEulers att = {
      ins_eulers.phi - ins_roll_neutral,
      ins_eulers.theta - ins_pitch_neutral,
      ins_eulers.psi
    };
    stateSetNedToBodyEulers_f(&att);
    stateSetBodyRates_f(&ins_rates);
#endif
    //uint8_t s = 4+VN100_REG_QMR_SIZE;
    //DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice,s,spi_buffer_input);
    vn100_trans.status = SPITransDone;
  }
  if (vn100_trans.status == SPITransFailed) {
    vn100_trans.status = SPITransDone;
    // FIXME retry config if not done ?
  }
}

static inline void parse_ins_msg(void)
{
  if (last_received_packet.ErrID != VN100_Error_None) {
    //TODO send error
    return;
  }

  // parse message (will work only with read and write register)
  switch (last_received_packet.RegID) {
    case VN100_REG_ADOR :
      ins_ador = last_received_packet.Data[0].UInt;
      break;
    case VN100_REG_ADOF :
      ins_adof = last_received_packet.Data[0].UInt;
      break;
    case VN100_REG_SBAUD :
      ins_baud = last_received_packet.Data[0].UInt;
      break;
    case VN100_REG_YPR :
      ins_eulers.phi   = RadOfDeg(last_received_packet.Data[2].Float);
      ins_eulers.theta = RadOfDeg(last_received_packet.Data[1].Float);
      ins_eulers.psi   = RadOfDeg(last_received_packet.Data[0].Float);
      break;
    case VN100_REG_QTN :
      ins_quat.qi = last_received_packet.Data[0].Float;
      ins_quat.qx = last_received_packet.Data[1].Float;
      ins_quat.qy = last_received_packet.Data[2].Float;
      ins_quat.qz = last_received_packet.Data[3].Float;
      float_eulers_of_quat(&ins_eulers, &ins_quat);
      break;
    case VN100_REG_QTM :
      ins_quat.qi = last_received_packet.Data[0].Float;
      ins_quat.qx = last_received_packet.Data[1].Float;
      ins_quat.qy = last_received_packet.Data[2].Float;
      ins_quat.qz = last_received_packet.Data[3].Float;
      float_eulers_of_quat(&ins_eulers, &ins_quat);
      ins_mag.x = last_received_packet.Data[4].Float;
      ins_mag.y = last_received_packet.Data[5].Float;
      ins_mag.z = last_received_packet.Data[6].Float;
      break;
    case VN100_REG_QTA :
      ins_quat.qi = last_received_packet.Data[0].Float;
      ins_quat.qx = last_received_packet.Data[1].Float;
      ins_quat.qy = last_received_packet.Data[2].Float;
      ins_quat.qz = last_received_packet.Data[3].Float;
      float_eulers_of_quat(&ins_eulers, &ins_quat);
      ins_accel.x = last_received_packet.Data[4].Float;
      ins_accel.y = last_received_packet.Data[5].Float;
      ins_accel.z = last_received_packet.Data[6].Float;
      break;
    case VN100_REG_QTR :
      ins_quat.qi = last_received_packet.Data[0].Float;
      ins_quat.qx = last_received_packet.Data[1].Float;
      ins_quat.qy = last_received_packet.Data[2].Float;
      ins_quat.qz = last_received_packet.Data[3].Float;
      float_eulers_of_quat(&ins_eulers, &ins_quat);
      ins_rates.p = last_received_packet.Data[4].Float;
      ins_rates.q = last_received_packet.Data[5].Float;
      ins_rates.r = last_received_packet.Data[6].Float;
      break;
    case VN100_REG_QMA :
      ins_quat.qi = last_received_packet.Data[0].Float;
      ins_quat.qx = last_received_packet.Data[1].Float;
      ins_quat.qy = last_received_packet.Data[2].Float;
      ins_quat.qz = last_received_packet.Data[3].Float;
      float_eulers_of_quat(&ins_eulers, &ins_quat);
      ins_mag.x = last_received_packet.Data[4].Float;
      ins_mag.y = last_received_packet.Data[5].Float;
      ins_mag.z = last_received_packet.Data[6].Float;
      ins_accel.x = last_received_packet.Data[7].Float;
      ins_accel.y = last_received_packet.Data[8].Float;
      ins_accel.z = last_received_packet.Data[9].Float;
      break;
    case VN100_REG_QAR :
      ins_quat.qi = last_received_packet.Data[0].Float;
      ins_quat.qx = last_received_packet.Data[1].Float;
      ins_quat.qy = last_received_packet.Data[2].Float;
      ins_quat.qz = last_received_packet.Data[3].Float;
      float_eulers_of_quat(&ins_eulers, &ins_quat);
      ins_accel.x = last_received_packet.Data[4].Float;
      ins_accel.y = last_received_packet.Data[5].Float;
      ins_accel.z = last_received_packet.Data[6].Float;
      ins_rates.p = last_received_packet.Data[7].Float;
      ins_rates.q = last_received_packet.Data[8].Float;
      ins_rates.r = last_received_packet.Data[9].Float;
      break;
    case VN100_REG_QMR :
      ins_quat.qi = last_received_packet.Data[0].Float;
      ins_quat.qx = last_received_packet.Data[1].Float;
      ins_quat.qy = last_received_packet.Data[2].Float;
      ins_quat.qz = last_received_packet.Data[3].Float;
      float_eulers_of_quat(&ins_eulers, &ins_quat);
      ins_mag.x = last_received_packet.Data[4].Float;
      ins_mag.y = last_received_packet.Data[5].Float;
      ins_mag.z = last_received_packet.Data[6].Float;
      ins_accel.x = last_received_packet.Data[7].Float;
      ins_accel.y = last_received_packet.Data[8].Float;
      ins_accel.z = last_received_packet.Data[9].Float;
      ins_rates.p = last_received_packet.Data[10].Float;
      ins_rates.q = last_received_packet.Data[11].Float;
      ins_rates.r = last_received_packet.Data[12].Float;
      break;
    case VN100_REG_YMR :
      ins_eulers.phi   = RadOfDeg(last_received_packet.Data[2].Float);
      ins_eulers.theta = RadOfDeg(last_received_packet.Data[1].Float);
      ins_eulers.psi   = RadOfDeg(last_received_packet.Data[0].Float);
      ins_mag.x = last_received_packet.Data[3].Float;
      ins_mag.y = last_received_packet.Data[4].Float;
      ins_mag.z = last_received_packet.Data[5].Float;
      ins_accel.x = last_received_packet.Data[6].Float;
      ins_accel.y = last_received_packet.Data[7].Float;
      ins_accel.z = last_received_packet.Data[8].Float;
      ins_rates.p = last_received_packet.Data[9].Float;
      ins_rates.q = last_received_packet.Data[10].Float;
      ins_rates.r = last_received_packet.Data[11].Float;
      break;
    default:
      break;
  }

}

#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

extern void vn100_report_task(void)
{
  DOWNLINK_SEND_AHRS_LKF(DefaultChannel, DefaultDevice,
                         &ins_eulers.phi, &ins_eulers.theta, &ins_eulers.psi,
                         &ins_quat.qi, &ins_quat.qx, &ins_quat.qy, &ins_quat.qz,
                         &ins_rates.p, &ins_rates.q, &ins_rates.r,
                         &ins_accel.x, &ins_accel.y, &ins_accel.z,
                         &ins_mag.x, &ins_mag.y, &ins_mag.z);
}

