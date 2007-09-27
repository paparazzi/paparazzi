


function [time, throttle, rpm, amp, thrust, torque] = read_mb_log(filename)

time=[];
throttle=[];
rpm=[];
amp=[];
thrust=[];
torque=[];

u=mopen(filename,'r');

while meof(u) == 0,
  line = mgetl(u, 1);
  if strindex(line, '#') ~= 1 & length(line) ~= 0,
    [nb_scan, _time, _throttle, _rpm, _amp, _thrust, _torque] = msscanf(1, line, '%f %f %f %f %f %f');
    if nb_scan == 6,
      time = [time _time];
      throttle = [throttle _throttle];
      rpm = [rpm _rpm];
      amp = [amp _amp];
      thrust = [thrust _thrust];
      torque = [torque _torque];
    end
  end
end

mclose(u);

endfunction
