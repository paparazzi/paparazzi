#ifndef BARO_SCP_H
#define BARO_SCP_H

#include "std.h"

#define SCP1000_OPERATION      0x03

#define SCP1000_HIGH_SPEED  0x09
#define SCP1000_HIGH_RES    0x0A
#define SCP1000_ULT_LOW_PW  0x0B
#define SCP1000_LOW_PW      0x0C

#define SCP1000_DATARD8     0x7F
#define SCP1000_DATARD16    0x80
#define SCP1000_TEMPOUT     0x81

#define BARO_SCP_UNINIT     0
#define BARO_SCP_IDLE       1
#define BARO_SCP_RD_TEMP    2
#define BARO_SCP_RD_PRESS_0 3
#define BARO_SCP_RD_PRESS_1 4

extern uint8_t  baro_scp_status;
extern uint32_t baro_scp_pressure;
extern uint16_t baro_scp_temperature;

void baro_scp_init(void);
void baro_scp_periodic(void);
void baro_scp_event(void);

#endif
