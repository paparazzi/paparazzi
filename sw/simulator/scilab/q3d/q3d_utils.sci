
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


function plot_with_min_rect(t, v, c, min_y, max_y)
  max_y = max(max_y, max(v));
  min_y = min(min_y, min(v));
  _rect = [t(1) min_y, t($), max_y];
  plot2d(t, v, c, rect=_rect);
endfunction




function draw_quad(i)

  global fdm_state;
  body_lines = list([-0.5 0; 0.5 0], [-0.8 0.08; -0.2 0.08], [0.2 0.08; 0.8 0.08]);
  dcmt = [ cos(fdm_state(FDM_STHETA,i)) -sin(fdm_state(FDM_STHETA,i))
           sin(fdm_state(FDM_STHETA,i)) cos(fdm_state(FDM_STHETA,i)) ];
  _rect = [ -1 -1 1 1];  
  earth_lines = list();
  plot2d(fdm_state(FDM_SX,i), fdm_state(FDM_SZ,i),3);
  for j=1:length(body_lines)
    earth_lines(j) = dcmt*body_lines(j)';
    plot2d(earth_lines(j)(1,:)+fdm_state(FDM_SX,i),earth_lines(j)(2,:)+fdm_state(FDM_SZ,i),1, rect=_rect);
  end
 
endfunction


function gen_video()

  dt_display = 1/25;
  
  time_display = 0;
  for i=1:length(fdm_time)
    if fdm_time(i) >= time_display
      set("current_figure",0);
      f=get("current_figure");
      f.figure_name="CTL";
      clf();
      drawlater();
      draw_quad(i);
//      filename = sprintf('images/frame_%03d.gif',i);
      filename = sprintf('images/frame_%03d.ppm',i);
//      xs2gif(0, filename);
      xs2ppm(0, filename);
      drawnow();
      pause
      time_display = time_display + dt_display;
    end
  end
  
endfunction

