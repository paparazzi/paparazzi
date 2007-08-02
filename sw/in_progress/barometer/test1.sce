


clear();
getf('baro_utils.sci');

filename = "data/07_07_26__16_37_09.baro.txt";
[time, gps_alt, pressure, gps_climb, temp] = baro_read_log(filename);


[p0, z0, a, b] = filter_init(10, pressure, gps_alt)

if 0
  deff('e=G(p,z)','a=p(1),b=p(2), gps=z(1), pressure=z(2), e=gps-(a/pressure+b)')
  deff('dg=dG(p,z)','a=p(1),b=p(2), gps=z(1), pressure=z(2), dg = [-1/pressure; -b]')
  p0 = [a, 0]';
  [p, err] = fit_dat(G, p0, [gps_alt; pressure], [1], dG) 

  a = p(1)
  b = p(2)
end

baro_alt_lin = a * pressure + b;
baro_alt_full = compute_altitude_exp(pressure);
//baro_alt = compute_altitude_lin(pressure);

xbasc();
if 0
  subplot(3,1,1)
  xtitle('Gps');
  plot2d([time], [gps_alt]);
  subplot(3,1,2)
  xtitle('Pressure');
  plot2d([time], [pressure]);
  subplot(3,1,3)
end
xtitle('Both');
plot2d([time; time; time]', [gps_alt; baro_alt_full; baro_alt_lin]', style=[5, 3, 2],  leg="gps@barofull@barolin");