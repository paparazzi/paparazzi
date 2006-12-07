%
% returns roll angle from an accelerometer reading
% under assumption that acceleration is vertical
%

function [phi] = phi_of_accel(accel)

phi = atan2(accel(2), accel(3));