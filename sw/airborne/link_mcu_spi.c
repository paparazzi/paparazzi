/*
 * $Id$
 *
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
#include "mcu_periph/spi.h"

struct link_mcu_msg link_mcu_from_ap_msg;
struct link_mcu_msg link_mcu_from_fbw_msg;

bool_t link_mcu_received;

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

void link_mcu_restart(void) {
  ComputeChecksum(link_mcu_from_fbw_msg);
  link_mcu_from_fbw_msg.checksum = crc;

  spi_buffer_input = (uint8_t*)&link_mcu_from_ap_msg;
  spi_buffer_output = (uint8_t*)&link_mcu_from_fbw_msg;
  spi_buffer_length = LINK_MCU_FRAME_LENGTH;
  SpiStart();
}

void link_mcu_event_task( void ) {
  if (spi_message_received) {
    /* Got a message on SPI. */
    spi_message_received = FALSE;

    /* A message has been received */
    ComputeChecksum(link_mcu_from_ap_msg);
    link_mcu_received = TRUE;
    if (link_mcu_from_ap_msg.checksum == crc)
      inter_mcu_received_ap = TRUE;
    else
      fbw_state->nb_err++;
  }
}

#endif /* FBW */



/*****************************************************************************/
#ifdef AP

uint8_t link_mcu_nb_err;
uint8_t link_mcu_fbw_nb_err;

void link_mcu_init(void) {
  link_mcu_nb_err = 0;
}

void link_mcu_send(void) {

  if (!SpiCheckAvailable()) {
    SpiOverRun();
    return;
  }

  ComputeChecksum(link_mcu_from_ap_msg);
  link_mcu_from_ap_msg.checksum = crc;
  spi_buffer_input = (uint8_t*)&link_mcu_from_fbw_msg;
  spi_buffer_output = (uint8_t*)&link_mcu_from_ap_msg;
  spi_buffer_length = LINK_MCU_FRAME_LENGTH;
  SpiSelectSlave0();
  SpiStart();
}

void link_mcu_event_task( void ) {
  if (spi_message_received) {
    /* Got a message on SPI. */
    spi_message_received = FALSE;
    /* A message has been received */
    ComputeChecksum(link_mcu_from_fbw_msg);
    if (link_mcu_from_fbw_msg.checksum == crc)
      inter_mcu_received_fbw = TRUE;
    else
      link_mcu_nb_err++;
  }
}

#endif /* AP */
