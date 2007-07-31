clear();
getf('rotations.sci');
getf('imu.sci');
getf('ahrs_euler_utils.sci');
getf('ahrs_quat_utils.sci');
getf('ekf.sci');

use_sim = 1;

if (use_sim),
  getf('quadrotor.sci');
  true_euler0 = [ 0.01; 0.25; 0.5]; 
  dt =  0.015625;
  [time, true_rates, true_eulers] = quadrotor_gen_roll_step(true_euler0, dt);
  [accel, mag, gyro] = imu_sim(time, true_rates, true_eulers);
else
  //filename = "../data/log_ahrs_test";
  //filename = "../data/log_ahrs_bug";
  filename = "../data/log_ahrs_roll";
  //filename = "../data/log_ahrs_yaw_pitched";
  [time, accel, mag, gyro] = imu_read_log(filename);
end


AHRS_STEP_PHI   = 0;
AHRS_STEP_THETA = 1;
AHRS_STEP_PSI   = 2;
AHRS_STEP_NB    = 3;

[m_eulers] = ahrs_compute_euler_measurements(accel, mag);

dt = time(2) - time(1);
[X0] = ahrs_quat_init(150, accel, mag);

// initial state covariance matrix
P0q = 1.;
P0b = .1;

P0 = [ P0q   0   0   0   0   0   0
	 0 P0q   0   0   0   0   0
	 0   0 P0q   0   0   0   0
	 0   0   0 P0q   0   0   0
	 0   0   0   0 P0b   0   0
         0   0   0   0   0 P0b   0
         0   0   0   0   0   0 P0b ];


Qq = 0.;
Qb = 0.008;

Q = [ Qq   0   0   0   0   0   0
       0  Qq   0   0   0   0   0
       0   0  Qq   0   0   0   0
       0   0   0  Qq   0   0   0
       0   0   0   0  Qb   0   0
       0   0   0   0   0  Qb   0
       0   0   0   0   0   0  Qb ];

X=[X0];
est_eulers = [euler_of_quat(X0(1:4))'];
//P=[P0];
P = P0;

for i=1:length(time)-1

  q0 = X(1, i);
  q1 = X(2, i);
  q2 = X(3, i);
  q3 = X(4, i);

  p = gyro(1,i) - X(5, i);
  q = gyro(2,i) - X(6, i);
  r = gyro(3,i) - X(7, i);
  
  OMEGA =   1/2 * [ 0  -p  -q  -r  0  0  0
                    p   0   r  -q  0  0  0
                    q  -r   0   p  0  0  0
                    r   q  -p   0  0  0  0
		    0   0   0   0  0  0  0
		    0   0   0   0  0  0  0
		    0   0   0   0  0  0  0 ];
  Xdot = OMEGA * X(:,i);

  F =   1/2 * [ 0  -p  -q  -r   q1  q2  q3;
                p   0   r  -q  -q0  q3 -q2;
                q  -r   0   p  -q3  q0  q1;
                r   q  -p   0   q2 -q1 -q0;
                0   0   0   0    0   0   0;
                0   0   0   0    0   0   0;
                0   0   0   0    0   0   0 ];

  [X1, P] = ekf_predict_continuous(X(:, i), Xdot, dt, P, F, Q);
  X1 = normalise_quat(X1);
//  X = [X X1];
  //  P = [P P1];
  est_euler = euler_of_quat(X1(1:4))';
  est_eulers = [est_eulers est_euler];

  ahrs_state = modulo(i, AHRS_STEP_NB);
  est_euler = euler_of_quat(X1);
  H = [0;0;0;0;0;0;0]';
  measure = 0;
  estimate = 0;
  
  select ahrs_state,
  case AHRS_STEP_PHI,
    measure = phi_of_accel(accel(:,i));
    estimate = est_euler(1);
    H = ahrs_quat_get_dphi_dX(X1);
    R = [1.3^2];
  case AHRS_STEP_THETA,
    measure = theta_of_accel(accel(:,i));
    estimate = est_euler(2);
    H = ahrs_quat_get_dtheta_dX(X1);
    R = [1.3^2];
  case AHRS_STEP_PSI,
    measure = psi_of_mag(m_eulers(1,i), m_eulers(2,i), mag(:,i));
    estimate = est_euler(3);
    H = ahrs_quat_get_dpsi_dX(X1);
    R = [2.5^2];
  end

  err = measure - estimate;
  [X2 P] = ekf_update(X1, P, H, R, err);

  X = [X X2];
  
  
  
  
end


xbasc();
subplot(3,1,1)
xtitle('Rates');
plot2d([time; time; time]', gyro', style=[5 3 2], leg="p@q@r");

subplot(3,1,2)
plot2d([time;time;time]', m_eulers', style=[5 3 2], leg="phi@theta@psi");
xtitle('Angles');
plot2d([time;time;time]', est_eulers', style=[0 0 0]);

subplot(3,1,3)
xtitle('Biases');
plot2d([time; time; time]', X(5:7, :)', style=[5 3 2], leg="p@q@r");
