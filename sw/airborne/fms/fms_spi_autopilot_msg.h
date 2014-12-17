/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#ifndef FMS_SPI_AUTOPILOT_MSG_H
#define FMS_SPI_AUTOPILOT_MSG_H
#include "downlink_transport.h"

int spi_ap_link_init(void);
void spi_ap_link_set_vane_callback(void (* vane_cb)(uint8_t vane_id, float alpha, float beta));
void spi_ap_link_set_pressure_absolute_callback(void (* pressure_cb)(uint8_t pressure_id, uint32_t pressure));
void spi_ap_link_set_pressure_differential_callback(void (* pressure_cb)(uint8_t pressure_id, uint32_t pressure));
void spi_ap_link_set_radio_control_callback(void (* radio_control_cb)(void));
void spi_ap_link_set_adc_callback(void (* adc_callback_fun)(uint16_t *adc_channels));
void spi_ap_link_periodic(void);
void spi_ap_link_downlink_send(struct DownlinkTransport *tp);

#ifdef USE_SPI_LINK
#define PERIODIC_SEND_EKF7_Y(_chan) spi_ap_link_downlink_send(_chan);
#endif

#endif // FMS_SPI_AUTOPILOT_MSG_H
