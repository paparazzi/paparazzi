#ifndef PROPS_CSC_H
#define PROPS_CSC_H

#include "generated/airframe.h"
#include "firmwares/rotorcraft/actuators.h"
#include "mcu_periph/sys_time.h"
#include "csc_ap_link.h"
#include "csc_msg_def.h"

extern uint8_t csc_prop_speeds[PROPS_NB];

void props_set(uint8_t i, uint8_t speed);
void props_commit();

#endif /* PROPS_CSC_H */
