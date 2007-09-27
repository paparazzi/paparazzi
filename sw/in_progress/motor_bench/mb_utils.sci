function [time, throttle, rpm, amp, thrust, torque] = read_mb_log(filename)

time=[];
rpm_meas=[];
throttle=[];
rpm=[];

no_line = 1;
u=mopen(filename,'r');
while meof(u) == 0,
  line = mgetl(u, 1);
  if strindex(line, '#') ~= 1 &  length(line) ~= 0,
    [nb_scan, t, c, p, p_f] = msscanf(1, line, '%f %f %f %f');
    if nb_scan == 4,
      time(no_line) = t;
      throttle(no_line) = c*max_rpm;
      rpm_sp(no_line) = p;
      rpm_meas(no_line) =p_f;
      no_line = no_line + 1;
    end
  end
end

mclose(u);
