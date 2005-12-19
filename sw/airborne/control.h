#ifndef CONTROL_H
#define CONTROL_H

#include <inttypes.h>
#include "inter_mcu.h"

extern int16_t desired_roll_dot;
extern float   roll_dot_pgain;
extern float   roll_dot_dgain;

extern int16_t desired_pitch_dot;
extern float   pitch_dot_pgain;
extern float   pitch_dot_dgain;

extern int16_t desired_yaw_dot;
extern float   yaw_dot_dgain;
extern float   yaw_dot_pgain;

extern int16_t desired_throttle;

extern pprz_t  control_commands[];

void control_run ( void );
void control_set_desired (  const pprz_t values[] );

#endif /* CONTROL_H */
