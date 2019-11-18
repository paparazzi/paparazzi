/*
 * Copyright (C) 2019 Tom van Dijk <tomvand@users.noreply.github.com>
 *
 * This code is based on the betaflight cc2500 and FrskyX implementation.
 * https://github.com/betaflight/betaflight
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

#ifndef RADIO_CONTROL_CC2500_FRSKY_SHARED_H
#define RADIO_CONTROL_CC2500_FRSKY_SHARED_H

#include <stdint.h>

typedef uint32_t timeDelta_t;

uint8_t listLength;
uint32_t missingPackets;
timeDelta_t timeoutUs;


// betaflight/src/main/rx/cc2500_frsky_shared.h  @ cb66ee0

/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

//#include "rx/rx_spi.h"

#define MAX_MISSING_PKT 100

#define DEBUG_DATA_ERROR_COUNT 0
#define DEBUG_DATA_MISSING_PACKETS 1
#define DEBUG_DATA_BAD_FRAME 2


#define SYNC_DELAY_MAX 9000

#define MAX_MISSING_PKT 100

enum {
    STATE_INIT = 0,
    STATE_BIND,
    STATE_BIND_TUNING,
    STATE_BIND_BINDING1,
    STATE_BIND_BINDING2,
    STATE_BIND_COMPLETE,
    STATE_STARTING,
    STATE_UPDATE,
    STATE_DATA,
    STATE_TELEMETRY,
    STATE_RESUME,
};

extern uint8_t listLength;
extern uint32_t missingPackets;
extern timeDelta_t timeoutUs;

void initialiseData(bool inBindState);

void nextChannel(uint8_t skip);


#endif // RADIO_CONTROL_CC2500_FRSKY_SHARED_H
