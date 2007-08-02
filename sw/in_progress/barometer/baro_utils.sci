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




function [time_pres, pres] = baro_read_pprz_log(filename)

time_pres= [];
pres     = [];

u=mopen(filename, 'r');
while meof(u) == 0,
  line = mgetl(u, 1);
  if (line == "") continue end
  [nb_scan, tip, ac, pr, te] = msscanf(1, line, '%f %d BARO_MS5534A %d %d');  
  if (nb_scan == 4)
    time_pres = [time_pres tip];
    pres = [pres pr];
  end

end
mclose(u);
endfunction


function [baro_alt] = compute_altitude_exp(baro_pressure)
P0 = 1013.25;
T0 = 288.15;
Tg =6.5/1000;
R  = 287.052;
g = 9.81;
baro_alt=[];
len = length(baro_pressure)
for i=1:len
  ba = T0/Tg*(1-(baro_pressure(i)/P0)^(Tg*R/g));
  baro_alt=[baro_alt ba];
end
endfunction

function [baro_alt] = compute_altitude_lin(baro_pressure)
baro_alt=[];
len = length(baro_pressure)
for idx=1:len
  p = 10 * baro_pressure(idx);
  if      (p > 10300)
    i = 1638.;
    j = -139.;
    pl = 10300.;
  elseif ( p > 9700 )
    i = 1720.;
    j = 365.;
    pl = 9700.;    
  elseif ( p > 9200 )
    i = 1802.;
    j = 805.;
    pl = 9200.;
  elseif ( p > 8500 )
    i = 1905.;
    j = 1456.;
    pl = 8500.;
  elseif ( p > 7800 )
    
  elseif ( p > 7100 )
    
  end
  ba = j - (p-pl) * i / 2^11;
  baro_alt = [baro_alt ba];
end

endfunction


//
// intersema application note 501 page 8
//
function [pres, alt, a, b] = filter_init(avg_len, pressure, gps_alt)

  avg_pressure = sum(pressure(1:avg_len), 'c') / avg_len;
  avg_gps = sum(gps_alt(1:avg_len), 'c') / avg_len;
  
  if (avg_pressure > 1030.0) 
    a = -16380. / 2^11;
  elseif (avg_pressure > 970.0)
    a = -17200. / 2^11;
  elseif (avg_pressure > 920.0)
    a = -18020. / 2^11;
  elseif (avg_pressure > 850.0)
    a = -19050. / 2^11;
  elseif (avg_pressure > 780.0)
    a = -20330. / 2^11;
  elseif (avg_pressure > 710.0)
    a = -21880. / 2^11;
  elseif (avg_pressure > 650.0)
    a = -23590. / 2^11;
  end;

  pres = avg_pressure;
  alt = avg_gps;
  b = avg_gps - a * avg_pressure;

endfunction



function [Pi] = baro_get_P(P, i)
  
  Pi = P(:, 2*i-1:2*i);

endfunction

