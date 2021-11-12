/*
 * Copyright (C) 2003-2005  Pascal Brisset, Antoine Drouin
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/intermcu/link_mcu_spi.h
 * Transport for the communication between FBW and AP via SPI.
 *
 */

#ifndef LINK_MCU_SPI_H
#define LINK_MCU_SPI_H

#include <inttypes.h>
#include "modules/intermcu/inter_mcu.h"
#include "mcu_periph/spi.h"

#ifndef SITL
#include "modules/intermcu/link_mcu_hw.h"
#endif

struct link_mcu_msg {
  union  {
    struct fbw_state from_fbw;
    struct ap_state  from_ap;
  } payload;
  uint16_t checksum;
};

extern struct link_mcu_msg link_mcu_from_ap_msg;
extern struct link_mcu_msg link_mcu_from_fbw_msg;

extern struct spi_transaction link_mcu_trans;

extern bool link_mcu_received;

extern void link_mcu_init(void);
extern void link_mcu_event_task(void);

#ifdef FBW
extern void link_mcu_restart(void);
#endif /* FBW */

#ifdef AP
extern uint8_t link_mcu_nb_err;
extern uint8_t link_mcu_fbw_nb_err;

extern void link_mcu_send(void);
#endif /* AP */

#endif /* LINK_MCU_SPI_H */
