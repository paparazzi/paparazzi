#ifndef BARO_SCP_H
#define BARO_SCP_H

#include "std.h"

#define BARO_SCP_UNINIT     0
#define BARO_SCP_IDLE       1
#define BARO_SCP_RD_TEMP    2
#define BARO_SCP_RD_PRESS_0 3
#define BARO_SCP_RD_PRESS_1 4

extern uint8_t  baro_scp_status;
extern uint32_t baro_scp_pressure;
extern uint16_t baro_scp_temperature;
extern bool_t   baro_scp_available;
extern volatile bool_t baro_scp_i2c_done;

void baro_scp_init(void);
void baro_scp_periodic(void);

#endif
