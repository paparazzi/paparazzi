function [raw_mag, raw_accel] = read_log(filename)
  
  raw_mag = [];
  raw_accel = [];

  u=mopen(filename,'r');
  
  while meof(u) == 0,
    line = mgetl(u, 1);
    if strindex(line, '#') ~= 1 & length(line) ~= 0,
      [nb_scan, _time, _mx, _my, _mz] = msscanf(1, line, '%f 151 IMU_MAG_RAW %f %f %f');
      if nb_scan == 4,
        raw_mag = [raw_mag [_mx _my _mz]'];
      else
        [nb_scan, _time, _ax, _ay, _az] = msscanf(1, line, '%f 149 IMU_ACCEL_RAW %f %f %f');    
        if nb_scan == 4,
	  raw_accel = [raw_accel [_ax _ay _az]'];
	end
      end
    end
  end
  
  mclose(u);
  
endfunction



function [fraw_sensor] = filter_noise(raw_sensor, window_size, max_var)

  [nl, nc] = size(raw_sensor);
  fraw_sensor = [];

  for i=window_size+1:nc-window_size-1 

    v = variance(raw_sensor(:,i-window_size:i+window_size),'c');
    
    if norm(v) < max_var
      fraw_sensor = [fraw_sensor raw_sensor(:,i)];
    end
  end

endfunction


function [scaled_sensor] = scale_sensor(raw_sensor, g, n)
  
  [nl, nc] = size(raw_sensor);
  scaled_sensor = zeros(nl, nc);
  
  for i=1:nc
    scaled_sensor(:,i) = g .* (raw_sensor(:,i) - n);
  end

endfunction

function [_mod] = compute_mod(sensor)

  [nl, nc] = size(sensor);
  _mod = zeros(1,nc);
  for i=1:nc
    _mod(i) = norm(sensor(:,i));
  end
  
endfunction
