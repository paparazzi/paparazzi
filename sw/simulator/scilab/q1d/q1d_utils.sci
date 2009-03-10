
function [rad] = rad_of_deg(deg)
  rad = deg / 180 * %pi;
endfunction

function [deg] = deg_of_rad(rad)
  deg = rad * 180 / %pi;
endfunction

function [o] = trim(i,_min, _max)
  o = i;
  if i > _max
    o = _max
  elseif i < _min
    o = _min
  end
endfunction

