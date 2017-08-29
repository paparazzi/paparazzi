#ifndef PID_H
#define PID_H

/* 
Function for a basic Proportional Integral Differential (PID) controller 
*/
extern float PID(float Kp, float Ki, float Kd, float err, float errprev, float *errint, float dt);

#endif
