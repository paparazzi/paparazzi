//
// X = [ z a ]
//
//


clear();
getf('baro_utils.sci');
getf('ekf.sci');

filename = "data/07_07_26__16_37_09.baro.txt";
[time, gps_alt, pressure, gps_climb, temp] = baro_read_log(filename);

do_predict = 1;
do_update = 1;
dt = (time(length(time)) - time(1)) / length(time)

//
// Initialisation
//
[pres0, alt0, a, b] = filter_init(10, pressure, gps_alt)

X0 = [ alt0; a];

P0 = [ 1.  0.
       0.  2. ];

X = [X0];
P = [P0];
   
//
// Iterations
//
for i=2:length(gps_alt)

  // prediction
  // initial state
  X0 = X(:, i-1);
  // initial covariance
  P0 = baro_get_P(P, i-1);
  // command
  delta_p = pressure(i) - pressure(i-1);
  delta_z = X0(2) * delta_p;
  X1 = X0 + [delta_z; 0];
  // jacobian
  F = [ 1.  delta_p
        0   1.       ];
  // process covariance noise
//  Q = [ 1.   0.
//        0.   0.1 ];
  Q = [ .0001   0.
        0.   0.0000001 ];
  P1 = F*P0*F' + Q;

  
  // update
  err = gps_alt(i) - X1(1);
  H = [1 0];
  R = 10;
  E = H * P1 * H' + R;
  K = P1 * H' * inv(E);
  P2 = P1 - K * H * P1;
  X2 = X1 + K * err;
  
  X = [X X2];
  P = [P P2];

  
end
   
   
   
//
// Display
//
xbasc();
subplot(3,1,1)
xtitle('altitude');
plot2d([time; time]', [gps_alt; X(1,:)]', style=[5, 3],  leg="gps@estimate");
subplot(3,1,2)
xtitle('a');
plot2d([time]', [X(2,:)]', style=[5], leg="a");
subplot(3,1,3)
xtitle('covariance');
P11 = [];
P22 = [];
for i=1:length(time)
  Pi = baro_get_P(P, i);
  P11 = [P11 Pi(1,1)];
  P22 = [P22 Pi(2,2)];
end
plot2d([time; time]', [P11; P22]', style=[5 3 2], leg="Pz@Pa");