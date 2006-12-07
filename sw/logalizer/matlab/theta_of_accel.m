%
% return pitch angle from an accelerometer reading
% under assumption that acceleration is vertical
%

function [theta] = theta_of_accel(accel)

theta = -asin( accel(1) / norm(accel));