//
//
//
//


clear();
getf('baro_utils.sci');
getf('ekf.sci');

filename = "data/07_08_02__14_57_30.data";
//filename = "data/07_08_02__15_19_47.data";
[time_pressure, pressure, time_altitude, altitude] = baro_read_pprz_log(filename);

//[time, gps_alt, pressure, gps_climb, temp] = baro_read_log(filename);


//
// Initialisation
//
time_start = 110.;
time_end = 250.;
//time_start = 100.;
//time_end = 150.;

[pressure0,end_pressure, altitude0, end_altitude,  a0, b0] = filter_init_timed(time_start, time_end, time_pressure, pressure, time_altitude, altitude)

X0 = [ altitude0; a0];

P0 = [ 1.  0.
       0.  2. ];
   
X = [X0];
P = [P0];
time_state = [time_pressure(end_pressure)];
//
// Iterations
//
idx_a = end_altitude;
idx_s = 2;
for idx_p=(end_pressure+1):length(time_pressure)

  // prediction
  // initial state
  X0 = X(:, idx_s-1);
  // initial covariance
  P0 = baro_get_P(P, idx_s-1);
  // command
  delta_p = pressure(idx_p) - pressure(idx_p-1);
  delta_z = X0(2) * delta_p;
  X1 = X0 + [delta_z; 0];
  // jacobian
  F = [ 1.  delta_p
        0   1.       ];
  // process covariance noise
  Q = [ 1e-4   0.
        0.   1e-6 ];
  P1 = F*P0*F' + Q;
  
  // update
  err = altitude(idx_a) - X1(1);
  H = [1 0];
  R = 25;
  E = H * P1 * H' + R;
  K = P1 * H' * inv(E);
  P2 = P1 - K * H * P1;
  X2 = X1 + K * err;
  
  X = [X X2];
  P = [P P2];
  time_state = [time_state time_pressure(idx_p)];
  idx_p = idx_p + 1;
  idx_s = idx_s + 1;
  while ((idx_p < length(time_pressure)) & ...
         (time_altitude(idx_a) < time_pressure(idx_p)) & ...
         (idx_a < length(time_altitude))), idx_a = idx_a + 1; end
  
end
   
   

dumb_alt = a0 * pressure + b0;


//
// Display
//
xbasc();
subplot(4,1,1)
xtitle('altitude');
plot2d([time_altitude]', [altitude]', style=[5]);
plot2d([time_state]', [X(1,:)]', style=[3, 5],  leg="est_alt@gps");
plot2d([time_pressure]', [dumb_alt]', style=[1]);
subplot(4,1,2)
xtitle('pressure');
plot2d([time_pressure]', [pressure]', style=[5], leg="pressure");
subplot(4,1,3)
xtitle('a');
plot2d([time_state]', [X(2,:)]', style=[3], leg="a");
subplot(4,1,4)
xtitle('covariance');
P11 = [];
P22 = [];
for i=1:length(time_state)
  Pi = baro_get_P(P, i);
  P11 = [P11 Pi(1,1)];
  P22 = [P22 Pi(2,2)];
end
plot2d([time_state; time_state]', [P11; P22]', style=[5 3], leg="Palt@Pa");