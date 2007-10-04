#ifndef MULTITILT_H
#define MULTITILT_H

#include "std.h"

#define MT_STATUS_UNINIT       0
#define MT_STATUS_INITIALISING 1
#define MT_STATUS_RUNNING      2
#define MT_STATUS_CRASHED      3

extern uint8_t mt_status;

extern float mt_phi; 
extern float mt_p;
extern float mt_bp;
extern float mt_P_phi[][];

extern void multitilt_reset(void);
extern void multitilt_run(void);



#endif /* MULTITILT_H */
