%
% initialise a DCM from a quaternion
%

function [dcm] = dcm_of_quat(quat)

q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);

dcm00 = q0^2 + q1^2 - q2^2 - q3^2;
dcm01 = 2 * (q1*q2 + q0*q3);
dcm02 = 2 * (q1*q3 - q0*q2);
dcm10 = 2 * (q1*q2 - q0*q3);
dcm11 = q0^2 - q1^2 + q2^2 - q3^2;
dcm12 = 2 * (q2*q3 + q0*q1);
dcm20 = 2 * (q1*q3 + q0*q2);
dcm21 = 2 * (q2*q3 - q0*q1);
dcm22 = q0^2 - q1^2 - q2^2 + q3^2;

dcm = [ dcm00 dcm01 dcm02
	dcm10 dcm11 dcm12
	dcm20 dcm21 dcm22 ];