%
% initialise euler angles from a quaternion
%

function [psi] = psi_of_quat(quat)

q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);

psi = atan2(2*(q1*q2 + q0*q3), (q0^2 + q1^2 - q2^2 - q3^2));


