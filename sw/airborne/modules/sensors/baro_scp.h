#ifndef BARO_SCP_H
#define BARO_SCP_H

#include "std.h"

#ifdef STM32
#error LPC21_only
#endif

#define STA_UNINIT       0
#define STA_INITIALISING 1
#define STA_IDLE         2

extern uint8_t  baro_scp_status;
extern uint32_t baro_scp_pressure;
extern uint16_t baro_scp_temperature;
extern bool baro_scp_available;

void baro_scp_init(void);
void baro_scp_periodic(void);
void baro_scp_event(void);

#endif
