%
% initialise euler angles from a quaternion
%

function [theta] = theta_of_quat(quat)

q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);

theta = asin(-2*(q1*q3 - q0*q2));


