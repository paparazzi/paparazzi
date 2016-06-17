/*
 * Copyright (C) 2003-2006  Pascal Brisset, Antoine Drouin
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

#include "link_mcu_spi.h"

#ifndef LINK_MCU_SPI_DEV
#define LINK_MCU_SPI_DEV spi1
#endif

struct link_mcu_msg link_mcu_from_ap_msg;
struct link_mcu_msg link_mcu_from_fbw_msg;

struct spi_transaction link_mcu_trans;

bool link_mcu_received;

static uint16_t crc = 0;

#define PAYLOAD_LENGTH sizeof(link_mcu_from_fbw_msg.payload)
#define LINK_MCU_FRAME_LENGTH sizeof(struct link_mcu_msg)

#define ComputeChecksum(_buf) { \
    uint8_t i; \
    crc = CRC_INIT; \
    for(i = 0; i < PAYLOAD_LENGTH; i++) { \
      uint8_t _byte = ((uint8_t*)&_buf)[i]; \
      crc = CrcUpdate(crc, _byte); \
    } \
  }

#ifdef FBW

void link_mcu_init(void)
{

  link_mcu_trans.cpol = SPICpolIdleLow;
  link_mcu_trans.cpha = SPICphaEdge2;
  link_mcu_trans.dss = SPIDss8bit;
  link_mcu_trans.input_buf = (uint8_t *)&link_mcu_from_ap_msg;
  link_mcu_trans.output_buf = (uint8_t *)&link_mcu_from_fbw_msg;
  link_mcu_trans.input_length = LINK_MCU_FRAME_LENGTH;
  link_mcu_trans.output_length = LINK_MCU_FRAME_LENGTH;
  spi_slave_register(&(LINK_MCU_SPI_DEV), &link_mcu_trans);
}

void link_mcu_restart(void)
{
  ComputeChecksum(link_mcu_from_fbw_msg);
  link_mcu_from_fbw_msg.checksum = crc;

  // wait for the next transaction
  spi_slave_wait(&(LINK_MCU_SPI_DEV));
}

void link_mcu_event_task(void)
{
  if (link_mcu_trans.status == SPITransSuccess) {
    /* Got a message on SPI. */
    link_mcu_trans.status = SPITransDone;

    /* A message has been received */
    ComputeChecksum(link_mcu_from_ap_msg);
    link_mcu_received = true;
    if (link_mcu_from_ap_msg.checksum == crc) {
      inter_mcu_received_ap = true;
    } else {
      fbw_state->nb_err++;
    }
  }
  if (link_mcu_trans.status == SPITransFailed) {
    link_mcu_trans.status = SPITransDone;
    link_mcu_received = true;
    fbw_state->nb_err++;
  }
}

#endif /* FBW */



/*****************************************************************************/
#ifdef AP

#ifndef LINK_MCU_SLAVE_IDX
#define LINK_MCU_SLAVE_IDX SPI_SLAVE0
#endif

uint8_t link_mcu_nb_err;
uint8_t link_mcu_fbw_nb_err;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_debug_link(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t mcu1_ppm_cpt_foo = 0; //FIXME
  pprz_msg_send_DEBUG_MCU_LINK(trans, dev, AC_ID,
                               &link_mcu_nb_err, &link_mcu_fbw_nb_err, &mcu1_ppm_cpt_foo);
}
#endif

void link_mcu_init(void)
{
  link_mcu_nb_err = 0;

  link_mcu_trans.cpol = SPICpolIdleLow;
  link_mcu_trans.cpha = SPICphaEdge2;
  link_mcu_trans.dss = SPIDss8bit;
  link_mcu_trans.select = SPISelectUnselect;
  link_mcu_trans.slave_idx = LINK_MCU_SLAVE_IDX;
  link_mcu_trans.input_buf = (uint8_t *)&link_mcu_from_fbw_msg;
  link_mcu_trans.output_buf = (uint8_t *)&link_mcu_from_ap_msg;
  link_mcu_trans.input_length = LINK_MCU_FRAME_LENGTH;
  link_mcu_trans.output_length = LINK_MCU_FRAME_LENGTH;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEBUG_MCU_LINK, send_debug_link);
#endif
}

void link_mcu_send(void)
{

  ComputeChecksum(link_mcu_from_ap_msg);
  link_mcu_from_ap_msg.checksum = crc;
  spi_submit(&(LINK_MCU_SPI_DEV), &link_mcu_trans);
}

void link_mcu_event_task(void)
{
  if (link_mcu_trans.status == SPITransSuccess) {
    /* Got a message on SPI. */
    link_mcu_trans.status = SPITransDone;
    /* A message has been received */
    ComputeChecksum(link_mcu_from_fbw_msg);
    if (link_mcu_from_fbw_msg.checksum == crc) {
      inter_mcu_received_fbw = true;
    } else {
      link_mcu_nb_err++;
    }
  }
  if (link_mcu_trans.status == SPITransFailed) {
    link_mcu_trans.status = SPITransDone;
    link_mcu_received = true;
    link_mcu_nb_err++;
  }
}

#endif /* AP */
