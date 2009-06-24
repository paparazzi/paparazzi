#ifndef BUSS_TWI_BLMC_HW_H
#define BUSS_TWI_BLMC_HW_H

#include <string.h>
#include "std.h"
#include "i2c.h"

#include "airframe.h"

#define BUSS_TWI_BLMC_STATUS_IDLE 0
#define BUSS_TWI_BLMC_STATUS_BUSY 1

void motors_init ( void );
void motors_set_motor(uint8_t id, int16_t value);
void motors_commit();
void motors_commit_next();


#define BussTwiBlmcNext() motors_commit_next();


#endif /* BUSS_TWI_BLMC_HW_H */
