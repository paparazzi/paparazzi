function [rad] = rad_of_deg(deg)
  rad = deg / 180 * %pi;
endfunction

function [deg] = deg_of_rad(rad)
  deg = rad * 180 / %pi;
endfunction

AXIS_X  = 1;
AXIS_Z  = 2;
AXIS_NB = 2;
