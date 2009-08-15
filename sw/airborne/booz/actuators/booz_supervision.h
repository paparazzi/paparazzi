#ifndef BOOZ_SUPERVISION_H
#define BOOZ_SUPERVISION_H

#include "std.h"

extern void supervision_init(void);
extern void supervision_run(bool_t motors_on, int32_t in_cmd[]);

extern int32_t supervision_commands[];

#endif /* BOOZ_SUPERVISION_H */
