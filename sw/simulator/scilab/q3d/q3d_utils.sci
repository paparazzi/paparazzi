
AXIS_X  = 1;
AXIS_Z  = 2;
AXIS_NB = 2;


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

function [_o] = trim_vect(_i,_min, _max)
  _o = _i;
  for i=1:length(_i)
    if _i(i) > _max
      _o(i) = _max
    elseif _i(i) < _min
      _o(i) = _min
    end
  end
endfunction
