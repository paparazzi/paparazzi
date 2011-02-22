/*
 * Paparazzi $Id: ins_xsens.c 3872 2009-08-05 14:42:41Z mmm $
 *
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
 * \brief driver for the VectorNav VN100 (Fixed-Wing part)
 */

#include "modules/ins/ins_vn100.h"
#include "mcu_periph/spi.h"
#include "estimator.h"
#include "generated/airframe.h"

#ifndef INS_YAW_NEUTRAL_DEFAULT
#define INS_YAW_NEUTRAL_DEFAULT 0.
#endif

void ins_init( void ) {

  ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
  ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
  ins_yaw_neutral = INS_YAW_NEUTRAL_DEFAULT;

  /* SPI polarity = 1 - data sampled on rising edge */
  SpiSetCPOL();
  /* SPI phase = 1 - SCK idle high */
  SpiSetCPHA();

  ins_ador = VN100_ADOR;
  ins_adof = VN100_ADOF;
  ins_baud = VN100_BAUD;

  ins_init_status = INS_VN100_SET_BAUD;

}

static inline bool_t ins_configure( void ) {
  switch (ins_init_status) {
    case INS_VN100_SET_BAUD :
      last_send_packet.RegID = VN100_REG_SBAUD;
      spi_buffer_length = 4+VN100_REG_SBAUD_SIZE;
      ins_init_status++;
      break;
    case INS_VN100_SET_ADOR :
      last_send_packet.RegID = VN100_REG_ADOR;
      spi_buffer_length = 4+VN100_REG_ADOR_SIZE;
      ins_init_status++;
      break;
    case INS_VN100_SET_ADOF :
      last_send_packet.RegID = VN100_REG_ADOF;
      spi_buffer_length = 4+VN100_REG_ADOF_SIZE;
      ins_init_status++;
      break;
    case INS_VN100_READY :
      return TRUE;
  }
  last_send_packet.CmdID = VN100_CmdID_WriteRegister;
  spi_buffer_input = (uint8_t*)&last_received_packet;
  spi_buffer_output = (uint8_t*)&last_send_packet;
  SpiSelectSlave0();
  SpiStart();
  return FALSE;
}

void ins_periodic_task( void ) {
  if (!SpiCheckAvailable()) {
    SpiOverRun();
    return;
  }

  if (!ins_configure()) return;

  // Fill request for QMR
  last_send_packet.CmdID = VN100_CmdID_ReadRegister;
  last_send_packet.RegID = VN100_REG_YMR;

  spi_buffer_input = (uint8_t*)&last_received_packet;
  spi_buffer_output = (uint8_t*)&last_send_packet;
  spi_buffer_length = 4+VN100_REG_YMR_SIZE;
  SpiSelectSlave0();
  SpiStart();

}

#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

void ins_event_task( void ) {
  if (spi_message_received) {
    spi_message_received = FALSE;
    parse_ins_msg();
#ifndef INS_VN100_READ_ONLY
    // Update estimator
    // FIXME Use a proper rotation matrix here
    EstimatorSetAtt((ins_eulers.phi - ins_roll_neutral), ins_eulers.psi, (ins_eulers.theta - ins_pitch_neutral));
#endif
    //uint8_t s = 4+VN100_REG_QMR_SIZE;
    //DOWNLINK_SEND_DEBUG(DefaultChannel,s,spi_buffer_input);
  }
}

