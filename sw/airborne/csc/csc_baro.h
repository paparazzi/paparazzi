#ifndef CSC_BARO_H
#define CSC_BARO_H

#include "std.h"

#define STA_UNINIT       0
#define STA_INITIALISING 1
#define STA_IDLE         2

extern uint8_t  baro_scp_status;
extern uint32_t baro_scp_pressure;
extern uint16_t baro_scp_temperature;
extern float baro_scp_alt;
extern bool_t baro_scp_available;

void baro_scp_init(void);
void baro_scp_periodic(void);

#define STA_UNINIT       0
#define STA_INITIALISING 1
#define STA_VALID         2

#define PERIODIC_SEND_BARO_MS5534A() DOWNLINK_SEND_BARO_MS5534A( \
  &baro_scp_pressure, \
  &baro_scp_temperature, \
  &baro_scp_alt \
  )

#endif
