 /*
 * Radio control over uavcan
 * Copyright (C) 2026 Fabien-B <fabien-b@github.com> 
 * This file is part of paparazzi. See LICENCE file.
 */

/** @file "modules/radio_control/radio_control_uavcan.h"
 * @author Fabien-B <Fabien-B@github.com>
 * Radio control from DroneCAN message `dronecan.sensors.rc.RCInput`.
 */

#pragma once

// Generated code holding the description of a given transmitter
#include "generated/radio.h"

#define RC_PPM_TICKS_OF_USEC(_x) (_x)
#define RC_PPM_SIGNED_TICKS_OF_USEC(_x) (_x)
#define USEC_OF_RC_PPM_TICKS(_x) (_x)

extern void rc_uavcan_init(void);
extern void rc_uavcan_event(void);
