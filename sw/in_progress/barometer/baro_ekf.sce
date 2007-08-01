//
//
//
//


clear();
getf('baro_utils.sci');
getf('ekf.sci');

filename = "data/07_07_26__16_37_09.baro.txt";
[time, gps_alt, pressure, gps_climb, temp] = baro_read_log(filename);

predict_only = 1;
dt = (time(length(time)) - time(1)) / length(time)

//
// Initialisation
//
[alt, a, b] = filter_init(10, pressure, gps_alt)

X0 = [ alt; a; b ];

P0 = [ 1. 0. 0.
       0. 1. 0.
       0. 0. 1. ];

X = [X0];
P = [P0];
   
//
// Iterations
//
for i=2:length(gps_alt)
  // initial state
  X0 = X(:, i-1);
  // initial covariance
  P0 = baro_get_P(P, i-1);
  // command
  pdot = (pressure(i) - pressure(i-1))/dt;
  climb = pdot * X0(2);
  // time derivative of state
  X0dot = [ climb; 0; 0];
  // Jacobian of Xdot wrt X
  F = [ 0 pdot  0
        0    0  0
	0    0  0 ];
  // process covariance noise
  Q = [10.   0.    0.
        0.   .1    0.
	0.   0.    0.1 ];

  [Xpred, Ppred] = ekf_predict_continuous(X0, X0dot, dt, P0, F, Q);
    
  if ( predict_only )
    X = [X Xpred];
    P = [P Ppred];
  else
    // measurement GPS
    measure = gps_alt(i);
    err = measure - Xpred(1);
    // Jacobian of measurement wrt X
    H = [ 1 pressure(i) 1];
    // Measurement covariance noise
    R = 10.;
    [Xup, Pup] = ekf_update(Xpred, Ppred, H, R, err);
  
    // measurement pressure
    measure = pressure(i);
    err
    
    X = [X Xup];
    P = [P Pup];
    
  end 
end
   
   
   
//
// Display
//
xbasc();
subplot(4,1,1)
xtitle('altitude');
plot2d([time; time]', [gps_alt; X(1,:)]', style=[5, 3],  leg="gps@estimate");
subplot(4,1,2)
xtitle('a');
plot2d([time]', [X(2,:)]', style=[5], leg="a");
subplot(4,1,3)
xtitle('b');
plot2d([time]', [X(3,:)]', style=[5], leg="b");
subplot(4,1,4)
xtitle('covariance');
P11 = [];
P22 = [];
P33 = [];
for i=1:length(time)
  Pi = baro_get_P(P, i);
  P11 = [P11 Pi(1,1)];
  P22 = [P22 Pi(2,2)];
  P33 = [P33 Pi(3,3)];
end
plot2d([time; time; time]', [P11; P22; P33]', style=[5 3 2], leg="alt@a@b");