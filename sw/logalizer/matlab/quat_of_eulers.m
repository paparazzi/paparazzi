
%
% initialise a quaternion from euler angles
%

function [quat] = quat_of_eulers(eulers)


phi2     = eulers(1) / 2.0;
theta2   = eulers(2) / 2.0;
psi2     = eulers(3) / 2.0;

sinphi2   = sin( phi2 );
cosphi2   = cos( phi2 );
sintheta2 = sin( theta2 );
costheta2 = cos( theta2 );
sinpsi2   = sin( psi2 );
cospsi2   = cos( psi2 );

q0 = cosphi2 * costheta2 * cospsi2 + sinphi2 * sintheta2 * sinpsi2;
q1 = sinphi2 * costheta2 * cospsi2 - cosphi2 * sintheta2 * sinpsi2;
q2 = cosphi2 * sintheta2 * cospsi2 + sinphi2 * costheta2 * sinpsi2;
q3 = cosphi2 * costheta2 * sinpsi2 - sinphi2 * sintheta2 * cospsi2;

quat = [q0 q1 q2 q3]';
