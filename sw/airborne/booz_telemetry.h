#ifndef BOOZ_TELEMETRY_H
#define BOOZ_TELEMETRY_H

#include "std.h"
#include "messages.h"
#include "periodic.h"
#include "uart.h"

#include "radio_control.h"
#include "link_imu.h"

#include "downlink.h"

#define PERIODIC_SEND_BOOZ_STATUS() DOWNLINK_SEND_BOOZ_STATUS(&link_imu_nb_err, &(link_imu_state.status), &rc_status)

extern uint8_t telemetry_mode_Main;

static inline void booz_telemetry_periodic_task(void) {
  PeriodicSendMain()
}


#endif /* BOOZ_TELEMETRY_H */
