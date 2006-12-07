%
% return yaw angle from a magnetometer reading, knowing roll and pitch
%
% The rotation matrix to rotate from NED frame to body frame without
% rotating in the yaw axis is:
%
% [ 1      0         0    ] [ cos(Theta)  0  -sin(Theta) ]
% [ 0  cos(Phi)  sin(Phi) ] [      0      1       0      ]
% [ 0 -sin(Phi)  cos(Phi) ] [ sin(Theta)  0   cos(Theta) ]
%
% This expands to:
%
% [  cos(Theta)              0      -sin(Theta)         ]
% [  sin(Phi)*sin(Theta)  cos(Phi)   sin(Phi)*cos(Theta)]
% [  cos(Phi)*sin(Theta)  -sin(Phi)  cos(Phi)*cos(Theta)]
%
% However, to untilt the compass reading, we need to use the 
% transpose of this matrix.
%
% [  cos(Theta)  sin(Phi)*sin(Theta)  cos(Phi)*sin(Theta) ]
% [      0       cos(Phi)            -sin(Phi)            ]
% [ -sin(Theta)  sin(Phi)*cos(Theta)  cos(Phi)*cos(Theta) ]
%

function [psi] = psi_of_mag(mag, phi, theta)

mn = cos(theta)            * mag(1)+ ...
     sin(phi) * sin(theta) * mag(2)+ ...
     cos(phi) * sin(theta) * mag(3);

me = cos(phi)*  mag(2) + ...
    -sin(phi) * mag(3);
    
psi =  -atan2( me, mn );