

#ifndef BOARDS_ARDRONE_ACTUATORS_H
#define BOARDS_ARDRONE_ACTUATORS_H

#include "paparazzi.h"

extern void actuators_init(void);
extern void actuators_set(pprz_t commands[]);
#define SetActuatorsFromCommands(commands) actuators_set(commands)

#endif /* BOARDS_ARDRONE_ACTUATORS_H */
