//
// State X = [z; a]
// X(k+1) = X(k) + [ a * delta_p; 0]
//
//


clear();
getf('baro_utils.sci');

//filename = "data/07_08_02__14_57_30.data";
//filename = "data/07_08_02__15_19_47.data";
//filename = "data/07_09_09__16_23_58.data";
filename = "data/test.data";

[time_pressure, pressure, time_altitude, altitude, time_accel, accel_x, accel_z] = baro_read_pprz_log(filename);

acc_neutral = 512;
acc_gain = 2. / 400 * 9.81;

ax_ms2 = (accel_x - acc_neutral) * acc_gain;
az_ms2 = (accel_z - acc_neutral) * acc_gain;

norm_accel = sqrt( ax_ms2^2 + az_ms2^2);
meas_pitch = atan(-az_ms2, ax_ms2); 
z_dot = [0.]

//
// Initialisation
//
time_start = 110.;
time_end = 250.;
//time_start = 100.;
//time_end = 150.;

[pressure0,end_pressure, altitude0, end_altitude,  a0, b0] = filter_init_timed(time_start, time_end, time_pressure, pressure, time_altitude, altitude)

zdot_0 = 0.;

X0 = [ altitude0; zdot_0];

P0 = [ 1.  0.
       0.  2. ];
   
X = [X0];
P = [P0];
time_state = [time_pressure(end_pressure)];
//
// Iterations
//
idx_a = end_altitude;
idx_acc = 1;
while idx_acc < length(time_accel) & time_accel(idx_acc) < time_pressure(end_pressure), idx_acc = idx_acc + 1; end
idx_s = 2;
for idx_p=(end_pressure+1):length(time_pressure)

  // prediction
  // initial state
  X0 = X(:, idx_s-1);
  // initial covariance
  P0 = baro_get_P(P, idx_s-1);
  // command
  delta_p = pressure(idx_p) - pressure(idx_p-1);
  delta_z = a0 * delta_p;
  X1 = X0 + [delta_z; 0];
  // jacobian
  F = [ 1.  delta_p
        0   1.       ];
  // process covariance noise
  Q = [ 1e-4   0.
        0.   1e-7 ];
  P1 = F*P0*F' + Q;
  
  // update
  z_dot_m = (norm_accel(idx_acc) - 9.81) * (time_accel(idx_acc) - time_accel(idx_acc - 1)) + X(2, idx_p-end_pressure); 
  z_dot = [z_dot z_dot_m];
  err =  z_dot_m - X1(2);
  H = [0 1];
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

   while ((idx_p < length(time_pressure)) & ...
         (time_accel(idx_acc) < time_pressure(idx_p)) & ...
       (idx_acc < length(time_accel))), idx_acc = idx_acc + 1; end
   
  
end
   
   

dumb_alt = a0 * pressure + b0;




//
// Display
//
xbasc();
subplot(6,1,1)
xtitle('altitude');
plot2d([time_altitude]', [altitude]', style=[5]);
plot2d([time_state]', [X(1,:)]', style=[3, 5],  leg="est_alt@gps");
plot2d([time_pressure]', [dumb_alt]', style=[1]);
subplot(6,1,2)
xtitle('pressure');
plot2d([time_pressure]', [pressure]', style=[5], leg="pressure");
subplot(6,1,3)
xtitle('z_dot');
plot2d([time_state]', [X(2,:)]', style=[3], leg="zdot_est");
plot2d([time_state]', [z_dot]', style=[3], leg="zdot_meas");
subplot(6,1,4)
xtitle('covariance');
P11 = [];
P22 = [];
for i=1:length(time_state)
  Pi = baro_get_P(P, i);
  P11 = [P11 Pi(1,1)];
  P22 = [P22 Pi(2,2)];
end
plot2d([time_state; time_state]', [P11; P22]', style=[5 3], leg="Pzt@Pzdot");


subplot(6,1,5)
plot2d([time_accel; time_accel; time_accel]', [ax_ms2; az_ms2; norm_accel]', style=[5 3 2], leg="Ax@Az@anorm");


subplot(6,1,6)
plot2d([time_accel]', [meas_pitch]', style=[5 3 2], leg="Ax@Az@anorm");
