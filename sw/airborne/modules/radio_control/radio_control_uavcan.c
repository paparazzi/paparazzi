 /*
 * Radio control over uavcan
 * Copyright (C) 2026 Fabien-B <fabien-b@github.com> 
 * This file is part of paparazzi. See LICENCE file.
 */

/** @file "modules/radio_control/radio_control_uavcan.c"
 * @author Fabien-B <Fabien-B@github.com>
 * Radio control from DroneCAN message `dronecan.sensors.rc.RCInput`.
 */

#include "modules/radio_control/radio_control_uavcan.h"
#include "modules/radio_control/radio_control.h"
#include "uavcan/uavcan.h"
#include "modules/core/abi.h"
#include "modules/core/threads.h"
#include "dronecan.sensors.rc.RCInput.h"
#include "generated/radio.h"

static uavcan_event rc_uavcan_ev;

struct dronecan_sensors_rc_RCInput rc_dronecan_msg;
static bool rc_frame_available;
static pprz_mutex_t rc_mtx;


static void rc_uavcan_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer) {
  if(pprz_mtx_trylock(&rc_mtx) == 0) {
    if(dronecan_sensors_rc_RCInput_decode(transfer, &rc_dronecan_msg)) {
      return; // decode error
    }
    rc_frame_available = true;
    pprz_mtx_unlock(&rc_mtx);
  }
}

void rc_uavcan_init(void)
{
  pprz_mtx_init(&rc_mtx);
  rc_frame_available = false;
  radio_control.nb_channel = 32;  // max value
  uavcan_bind(DRONECAN_SENSORS_RC_RCINPUT_ID, DRONECAN_SENSORS_RC_RCINPUT_SIGNATURE, &rc_uavcan_ev, &rc_uavcan_cb);
}


void rc_uavcan_event(void)
{
  bool send_abi = false;
  if(rc_frame_available) {
    if(pprz_mtx_trylock(&rc_mtx) == 0) {
      if((rc_dronecan_msg.status & DRONECAN_SENSORS_RC_RCINPUT_STATUS_FAILSAFE) ||
         (rc_dronecan_msg.status & DRONECAN_SENSORS_RC_RCINPUT_STATUS_QUALITY_VALID && rc_dronecan_msg.quality == 0)) {
        // RC LOST
      } else {
        radio_control.nb_channel = rc_dronecan_msg.rcin.len;
        radio_control.frame_cpt++;
        radio_control.time_since_last_frame = 0;
        radio_control.radio_ok_cpt = 0;
        radio_control.status = RC_OK;

        NormalizePpmIIR(rc_dronecan_msg.rcin.data, radio_control);
        
        send_abi = true;
      }
      
      rc_frame_available = false;
      pprz_mtx_unlock(&rc_mtx);
    }

    if(send_abi) {
      AbiSendMsgRADIO_CONTROL(RADIO_CONTROL_UAVCAN_ID, &radio_control);
    }
  }
}


