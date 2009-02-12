


function [time, gyro_raw, turntable] = read_gyro_log(ac_id, tt_id, filename)
  fmt_gyro = sprintf('%%f %d IMU_GYRO_RAW %%f %%f %%f', ac_id);
  fmt_tt   = sprintf('%%f %d IMU_TURNTABLE %%f', tt_id);
  time = [];
  gyro_raw = [];
  turntable = [];
  tt = 0;
  u=mopen(filename,'r');
  while meof(u) == 0,
    line = mgetl(u, 1);
    if strindex(line, '#') ~= 1 & length(line) ~= 0,
      [nb_scan, _time, _tt] = ...
	  msscanf(1, line, fmt_tt);
      if nb_scan == 2
	tt = _tt;
      end
      [nb_scan, _time, _rgx, _rgy, _rgz] = ...
	  msscanf(1, line, fmt_gyro);
      if nb_scan == 4
	time = [time _time];
	gyro_raw = [gyro_raw [_rgx; _rgy; _rgz]];
	turntable = [turntable tt];
      end
    end
  end  
  mclose(u);
endfunction




function [time, sensor_raw] = read_log_sensor_raw(ac_id, sensor_name, filename)

  fmt = sprintf('%%f %d IMU_GYRO_RAW %%f %%f %%f', ac_id, sensor_name);
  time = [];
  sensor_raw = [];
  u=mopen(filename,'r');
  while meof(u) == 0,
    line = mgetl(u, 1);
    if strindex(line, '#') ~= 1 & length(line) ~= 0,
      [nb_scan, _time, _rax, _ray, _raz] = ...
	  msscanf(1, line, fmt);
      if nb_scan == 4
	time = [time _time];
	sensor_raw = [sensor_raw [_rax; _ray; _raz]];
      end
    end
  end
  mclose(u);
  
endfunction

function [p] = min_max_calib(sensor_raw, ref_norm)
  min_raw = min(sensor_raw, 'c');
  max_raw = max(sensor_raw, 'c');
  neutral = (min_raw + max_raw) / 2;
  sensitivity = 2 * [ref_norm; ref_norm; ref_norm] ./ (max_raw - min_raw);
  p = [ sensitivity; neutral ];
endfunction

function [p, err] = datafit_calib(sensor_raw, ref_norm, p0)
  function e = err_norm(p, z)
    g = p(1:3);
    n = p(4:6);
    a = g .* (z-n);
    e = ref_norm^2 - a(1)^2 - a(2)^2 - a(3)^2;
  endfunction  
  [p, err] = datafit(err_norm, sensor_raw, p0, 'gc');
endfunction


function display_raw_sensors(time, sensor_raw)
  plot2d([time; time; time]', sensor_raw', leg="X@Y@Z");
  xtitle('raw sensor');
endfunction


function display_calib_sensor(time, _comp, _norm, lab, ref)

  subplot(2,1,1);
  plot2d(time, ref * ones(1,length(time)));
  plot2d(time, -ref * ones(1,length(time)));
  plot2d(time, zeros(1,length(time)));
  plot2d([time; time; time]', _comp', leg="X@Y@Z");

  xtitle("Components:"+lab);
  subplot(2,1,2);
  plot2d(time, ref * ones(1,length(time)));
//  plot2d3(time, _norm, 2);
  plot2d(time, _norm, 2);
  xtitle("Norme:"+lab);
  
endfunction


function [time_f, sensor_f] = filter_noisy_data(time_raw, sensor_raw, threshold, size_avg)

  sensor_f = [];
  time_f = [];
  for i=size_avg+1:length(sensor_raw(1,:))-size_avg
    s = variance(sensor_raw(:, i-size_avg:i+size_avg), 'c');
    if norm(s) < threshold
      sensor_f = [sensor_f sensor_raw(:,i)];
      time_f = [time_f time(i)];
    end
  end
  
endfunction


function [ac, gc] = apply_scaling(gain, neutral, ar)

  ac = [];
  gc = [];
  for i=1:length(ar(1,:))
    _ac = (ar(:,i) - neutral) .* gain;
  ac = [ac _ac];
  _gc = sqrt(_ac(1)^2 + _ac(2)^2 + _ac(3)^2);
  gc = [gc _gc];
end

endfunction


function print_xml(s_name, p, resolution)

printf("\n");
printf("<define name=""%s_X_SENS"" value=""%.8f"" integer=""16""/>\n", s_name, -p(1)*resolution);
printf("<define name=""%s_Y_SENS"" value=""%.8f"" integer=""16""/>\n", s_name,  p(2)*resolution);
printf("<define name=""%s_Z_SENS"" value=""%.8f"" integer=""16""/>\n", s_name, -p(3)*resolution);
printf("\n");
printf("<define name=""%s_X_NEUTRAL"" value=""%d""/>\n", s_name, round(p(4)));
printf("<define name=""%s_Y_NEUTRAL"" value=""%d""/>\n", s_name, round(p(5)));
printf("<define name=""%s_Z_NEUTRAL"" value=""%d""/>\n", s_name, round(p(6)));

endfunction

