#ifndef RADIO_CONTROL_H
#define RADIO_CONTROL_H

#include "config.h"
#include "types.h"
#include "ppm.h"
#include "radio.h"

#define MAX_PPRZ 9600
#define MIN_PPRZ -MAX_PPRZ
#define RC_AVG_PERIOD 8
#define RC_LOST_TIME 10
#define RC_REALLY_LOST_TIME 20

#define RC_OK          0
#define RC_LOST        1
#define RC_REALLY_LOST 2

extern int32_t rc_values[RADIO_CTL_NB];
extern uint8_t rc_status;

void radio_control_init ( void );
void radio_control_periodic_task ( void );
void radio_control_process_ppm ( void );

#endif /* RADIO_CONTROL_H */
