#include "std.h"
#include "actuators.h"
#include "servos_csc.h"
#include "csc_msg_def.h"

/* 40 Hz */
#ifndef SERVOS_PERIOD
#define SERVOS_PERIOD SERVOS_TICS_OF_USEC(25000)
#endif

void actuators_init ( void )
{

}
