function [time, gps_alt, pressure, gps_climb, temp] = baro_read_log(filename)

time=[];
gps_alt=[];
pressure=[];
gps_climb=[];
temp=[];

u=mopen(filename, 'r');
idx = 0;
while meof(u) == 0,
  line = mgetl(u, 1);
  [nb_scan, ti, ga, p, gc, te] = msscanf(1, line, '%f %f %f %f %f');  
  time = [time ti];
  gps_alt = [gps_alt ga];
  pressure = [pressure p];
  gps_climb = [gps_climb gc];
  temp = [temp te];
 end
mclose(u);

endfunction 