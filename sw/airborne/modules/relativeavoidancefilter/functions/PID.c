#include "PID.h"
/* 
Function for a basic Proportional Integral Differential (PID) controller 
*/
float PID(float Kp, float Ki, float Kd, float err, float errprev, float *errint, float dt)
{
	float errdev;
	*errint += err*dt;
	errdev = (err - errprev);
	
	return Kp * err + Ki * (*errint) + Kd * errdev;
}