/*
 * Copyright (C) 2014 Eduardo Lavratti (AGRESSiVA)
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
 * @file modules/sensors/flow_meter.c
 * Measure flow sensor pulse using external trigger pulse and
 * sends a message with the info.
 * 
 */


#include "flow_meter.h"
#include "modules/sensors/flow_meter_hw.h"
#include "subsystems/gps.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#ifndef FLOW_PULSE_PER_LITRE 
#define FLOW_PULSE_PER_LITRE 10500
#endif
float flow_sens;
  
void flow_init ( void ) {
  flow_sens = 1.0f / FLOW_PULSE_PER_LITRE;
  flow_hw_init();
}

void flow_periodic( void ) {
  if (flow_valid == TRUE) {
    uint32_t pulses, mililitres;

    mililitres = (uint32_t)(flow_sens * flow_pulse);
    pulses = flow_pulse;

    DOWNLINK_SEND_FLOW_METER(DefaultChannel, DefaultDevice,
                 &pulses,
                 &mililitres );
    flow_valid = FALSE;
  }
}

