/*
 * Copyright (C) 2006- Pascal Brisset, Antoine Drouin
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

/**
 * @file firmwares/fixedwing/fbw_downlink.h
 *
 * Set of macros defining the periodic telemetry messages of FBW process.
 *
 * The PeriodicSendAp() macro is generated from the telemetry description
 * (named in conf.xml, usually in conf/telemetry directory). This macro
 * is a sequence of calls to PERIODIC_SEND_message() which have to be defined
 * in the present file.
 *
 */

#ifndef FBW_DOWNLINK_H
#define FBW_DOWNLINK_H

#include <inttypes.h>
#include "messages.h"
#include "generated/periodic_telemetry.h"
#include "generated/airframe.h"
#include "subsystems/commands.h"
#include "subsystems/actuators.h"

#include "mcu_periph/uart.h"
#include "firmwares/fixedwing/main_fbw.h"
#include "subsystems/radio_control.h"
#include "subsystems/electrical.h"
#include "inter_mcu.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_FBW_DEVICE
#endif
#include "subsystems/datalink/downlink.h"

#define PERIODIC_SEND_COMMANDS(_trans, _dev) DOWNLINK_SEND_COMMANDS(_trans, _dev, COMMANDS_NB, commands)

#ifdef RADIO_CONTROL
#define PERIODIC_SEND_FBW_STATUS(_trans, _dev) DOWNLINK_SEND_FBW_STATUS(_trans, _dev, &(radio_control.status), &(radio_control.frame_rate), &fbw_mode, &electrical.vsupply, &electrical.current)
#ifdef RADIO_CONTROL_TYPE_PPM
#define PERIODIC_SEND_PPM(_trans, _dev) {                           \
  uint16_t ppm_pulses_usec[RADIO_CONTROL_NB_CHANNEL];        \
  for (int i=0;i<RADIO_CONTROL_NB_CHANNEL;i++)               \
    ppm_pulses_usec[i] = USEC_OF_RC_PPM_TICKS(ppm_pulses[i]); \
  DOWNLINK_SEND_PPM(_trans, _dev,                            \
                    &radio_control.frame_rate,               \
                    PPM_NB_CHANNEL,                          \
                    ppm_pulses_usec);                        \
}
#elif defined RADIO_CONTROL_TYPE_SBUS
#include "subsystems/radio_control/sbus.h"
#define PERIODIC_SEND_PPM(_trans, _dev) {                           \
  DOWNLINK_SEND_PPM(_trans, _dev,                            \
                    &radio_control.frame_rate,               \
                    SBUS_NB_CHANNEL,                          \
                    sbus.pulses);                        \
}
#else
#define PERIODIC_SEND_PPM(_trans, _dev) {}
#endif
#define PERIODIC_SEND_RC(_trans, _dev) DOWNLINK_SEND_RC(_trans, _dev, RADIO_CONTROL_NB_CHANNEL, radio_control.values)
#else // RADIO_CONTROL
#define PERIODIC_SEND_FBW_STATUS(_trans, _dev) { uint8_t dummy = 0; DOWNLINK_SEND_FBW_STATUS(_trans, _dev, &dummy, &dummy, &fbw_mode, &electrical.vsupply, &electrical.current); }
#define PERIODIC_SEND_PPM(_trans, _dev) {}
#define PERIODIC_SEND_RC(_trans, _dev) {}
#endif // RADIO_CONTROL

#ifdef ACTUATORS
#define PERIODIC_SEND_ACTUATORS(_trans, _dev) DOWNLINK_SEND_ACTUATORS(_trans, _dev, ACTUATORS_NB, actuators)
#else
#define PERIODIC_SEND_ACTUATORS(_trans, _dev) {}
#endif

#ifdef BRICOLAGE_ADC
extern uint16_t adc0_val[];

#define PERIODIC_SEND_ADC(_trans, _dev) {			\
    static const uint8_t mcu = 0;			\
    DOWNLINK_SEND_ADC(_trans, _dev, &mcu, 8, adc0_val);	\
  }
#else
#define PERIODIC_SEND_ADC(_trans, _dev) {}
#endif

static inline void fbw_downlink_periodic_task(void) {
  PeriodicSendFbw(DefaultChannel,DefaultDevice)
}


#endif /* FBW_DOWNLINK_H */
