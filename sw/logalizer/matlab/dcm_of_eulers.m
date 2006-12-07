%
% initialise a DCM from a set of eulers
%

function [dcm] = dcm_of_eulers(eulers)

phi = eulers(1);
theta = eulers(2);
psi = eulers(3);

dcm00 = cos(theta) * cos(psi);
dcm01 = cos(theta) * sin(psi);
dcm02 = -sin(theta);
dcm10 = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
dcm11 = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
dcm12 = sin(phi) * cos(theta);
dcm20 = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);
dcm21 = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);
dcm22 = cos(phi) * cos(theta);

dcm = [ dcm00 dcm01 dcm02
	dcm10 dcm11 dcm12
	dcm20 dcm21 dcm22 ];