#ifndef BOOZ_TELEMETRY_H
#define BOOZ_TELEMETRY_H

#include "std.h"
#include "messages.h"
#include "periodic.h"
#include "uart.h"

#include "radio_control.h"
#include "link_imu.h"
#include "booz_estimator.h"

#include "downlink.h"

#define PERIODIC_SEND_BOOZ_STATUS() DOWNLINK_SEND_BOOZ_STATUS(&link_imu_nb_err, &link_imu_status, &rc_status)
#define PERIODIC_SEND_BOOZ_FD()     DOWNLINK_SEND_BOOZ_FD(&booz_estimator_p, &booz_estimator_q, &booz_estimator_r,\
                          &booz_estimator_phi, &booz_estimator_theta, &booz_estimator_psi); 

extern uint8_t telemetry_mode_Main;

static inline void booz_telemetry_periodic_task(void) {
  PeriodicSendMain()
}


#endif /* BOOZ_TELEMETRY_H */
