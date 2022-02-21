#ifndef PERIODIC_TELEMETRY_H
#define PERIODIC_TELEMETRY_H

#include "std.h"
#include "generated/airframe.h"
#include "modules/datalink/telemetry_common.h"

#define TELEMETRY_FREQUENCY 60
#define TELEMETRY_PPRZ_NB_MSG 1
#define TELEMETRY_PPRZ_CBS { 0 }

static inline void periodic_telemetry_send_Ap(struct periodic_telemetry *telemetry, struct transport_tx *trans, struct link_device *dev) {
  (void)telemetry;
  (void)trans;
  (void)dev;
}

static inline void periodic_telemetry_send_Main(struct periodic_telemetry *telemetry, struct transport_tx *trans, struct link_device *dev) {
  (void)telemetry;
  (void)trans;
  (void)dev;
}

static inline void periodic_telemetry_send_Fbw(struct periodic_telemetry *telemetry, struct transport_tx *trans, struct link_device *dev) {
  (void)telemetry;
  (void)trans;
  (void)dev;
}

static inline void periodic_telemetry_send_InterMCU(struct periodic_telemetry *telemetry, struct transport_tx *trans, struct link_device *dev) {
  (void)telemetry;
  (void)trans;
  (void)dev;
}

#endif // PERIODIC_TELEMETRY_H
