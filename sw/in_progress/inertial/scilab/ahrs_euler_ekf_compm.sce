clear();
getf('rotations.sci');
getf('imu.sci');
getf('ahrs_euler_utils.sci');
getf('ekf.sci');

use_sim = 1;

if (use_sim),
  getf('quadrotor.sci');
  true_euler0 = [ 0.01; 0.2; 0.4]; 
  dt =  0.015625;
  [time, true_rates, true_eulers] = quadrotor_gen_roll_step(true_euler0, dt);
  [accel, mag, gyro] = imu_sim(time, true_rates, true_eulers);
else
  //filename = "../data/log_ahrs_test";
  filename = "../data/log_ahrs_bug";
  //filename = "../data/log_ahrs_roll";
  //filename = "../data/log_ahrs_yaw_pitched";
  [time, accel, mag, gyro] = imu_read_log(filename);
end

dt = time(2) - time(1);
[X0] = ahrs_euler_init(150, accel, mag);

// initial state covariance
P0e = 1.;
P0b = 1.;

P0 = [ P0e   0   0   0   0   0
	 0 P0e   0   0   0   0
	 0   0 P0e   0   0   0
	 0   0   0  P0b  0   0
         0   0   0   0  P0b  0
         0   0   0   0   0  P0b ];

// process covariance noise 
Qe = 0.;
Qb = 0.008;

Q = [ Qe   0   0   0   0   0
       0  Qe   0   0   0   0
       0   0  Qe   0   0   0
       0   0   0  Qb   0   0
       0   0   0   0  Qb   0
       0   0   0   0   0  Qb ];

// measure covariance noise
R = [
      1.3^2  0.     0.  
      0.     1.3^2  0.
      0.     0.     2.5^2
    ];

X=[X0];
P=[P0];
M=[X0(1:3)];

for i=2:length(time)
  // command
  rate_i_ = gyro(:,i-1) - X(4:6,i-1);
  // state
  Xi_ = X(:, i-1);
  // state time derivative
  Xdoti_ = ahrs_euler_get_Xdot(Xi_, rate_i_);
  // process covariance
  Pi_ = ahrs_euler_get_P(P, i-1);
  // Jacobian of state time derivative wrt state
  Fi_ = ahrs_euler_get_F(Xi_, rate_i_);
   
  [Xpred, Ppred] = ekf_predict_continuous(Xi_, Xdoti_, dt, Pi_, Fi_, Q);
  
//  X = [X Xpred];
//  P = [P Ppred];
  
  measure_i1 = euler_of_accel_mag(accel(:, i), mag(:, i));

  M = [M measure_i1];
  
  err = measure_i1 - Xpred(1:3);
  
  H = ahrs_euler_compm_get_deuler_dX(Xpred);
  
  [Xup, Pup] = ekf_update(Xpred, Ppred, H, R, err);
  
  X = [X Xup];
  P = [P Pup]; 
  
  
end

ahrsh_euler_display(time, X, P, M)
