clear();
getf('rotations.sci');
getf('imu.sci');
getf('ahrs_euler_utils.sci');
getf('ekf.sci');

use_sim = 1;
predict_only = 0;
predict_discrete = 0;
use_state_for_psi = 0;

if (use_sim),
  rand('seed', 0);
  getf('quadrotor.sci');
  true_euler0 = [ 0.01; 0.2; 0.4]; 
  dt =  0.015625;
  [time, true_rates, true_eulers] = quadrotor_gen_roll_step(true_euler0, dt);
  [accel, mag, gyro] = imu_sim(time, true_rates, true_eulers);
else
  //filename = "../data/log_ahrs_test";
  //filename = "../data/log_ahrs_bug";
  //filename = "../data/log_ahrs_roll";
  filename = "../data/log_ahrs_yaw_pitched";
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
Qe = 0.0;
Qb = 0.008;

Q = [ Qe   0   0   0   0   0
       0  Qe   0   0   0   0
       0   0  Qe   0   0   0
       0   0   0  Qb   0   0
       0   0   0   0  Qb   0
       0   0   0   0   0  Qb ];

// measurement covariance noise
R = [
      1.3^2  0.     0.  
      0.     1.3^2  0.
      0.     0.     2.5^2
    ];

X=[X0];       // state
P=[P0];       // state covariance
M=[X0(1:3)];  // measurements


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
   
  if predict_discrete
    [Xpred, Ppred] = ekf_predict_discrete(Xi_, Xdoti_, dt, Pi_, Fi_, Q);  
  else
    [Xpred, Ppred] = ekf_predict_continuous(Xi_, Xdoti_, dt, Pi_, Fi_, Q);
  end
  
  if use_state_for_psi
    m_phi = phi_of_accel(accel(:, i));
    m_theta = theta_of_accel(accel(:, i));
    m_psi = psi_of_mag(Xpred(1), Xpred(2), mag(:, i));
    measure_i1 = [ m_phi; m_theta; m_psi];
  else
    measure_i1 = euler_of_accel_mag(accel(:, i), mag(:, i));
  end

  M = [M measure_i1];
    
  if  predict_only, 
    X = [X Xpred];
    P = [P Ppred];
  else
    err = measure_i1 - Xpred(1:3);
  
    H = ahrs_euler_compm_get_deuler_dX(Xpred);
  
    [Xup, Pup] = ekf_update(Xpred, Ppred, H, R, err);
  
    X = [X Xup];
    P = [P Pup]; 
  end
  
end

ahrs_euler_display(time, X, P, M)
