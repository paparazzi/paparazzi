
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




function draw_quad(i, _rect)

  global fdm_state;
  body_lines = list([-0.25 0; 0.25 0], [-0.37 0.04; -0.13 0.04], [0.13 0.04; 0.37 0.04]);
  dcmt = [ cos(fdm_state(FDM_STHETA,i)) -sin(fdm_state(FDM_STHETA,i))
           sin(fdm_state(FDM_STHETA,i)) cos(fdm_state(FDM_STHETA,i)) ];
  earth_lines = list();
  plot2d(fdm_state(FDM_SX,1:i), fdm_state(FDM_SZ,1:i),3);
  for j=1:length(body_lines)
    earth_lines(j) = dcmt*body_lines(j)';
    plot2d(earth_lines(j)(1,:)+fdm_state(FDM_SX,i),earth_lines(j)(2,:)+fdm_state(FDM_SZ,i),1, rect=_rect);
  end

endfunction


function gen_video()
  global fdm_state;
  margin = 0.30;
  min_x = min(fdm_state(FDM_SX,:))-margin;
  max_x = max(fdm_state(FDM_SX,:))+margin;
  min_z = min(fdm_state(FDM_SZ,:))-margin;
  max_z = max(fdm_state(FDM_SZ,:))+margin;
  _rect = [min_x min_z max_x max_z];

  dt_display = 1/25;

  time_display = 0;
  frame_idx = 1;
  for i=1:length(fdm_time)
    if fdm_time(i) >= time_display
      set("current_figure",0);
      f=get("current_figure");
      f.figure_name="CTL";
      clf();
      drawlater();
      draw_quad(i, _rect);
      drawnow();
      if 0
	filename = sprintf('images/frame_%04d.ppm',frame_idx);
	xs2ppm(0, filename, 1);
      else
	filename = sprintf('images/frame_%04d.gif',i);
	xs2gif(0, filename);
      end
//      pause
      time_display = time_display + dt_display;
      frame_idx = frame_idx + 1;
    end
  end

endfunction

