
function [quat, biases] = ahrs(status, gyro, accel, mag)

AHRS_UNINIT     = 0;
AHRS_STEP_PHI   = 1;
AHRS_STEP_THETA = 2;
AHRS_STEP_PSI   = 3;

persistent ahrs_quat;
persistent ahrs_biases;
persistent ahrs_rates;
persistent ahrs_P;     % covariance matrix
persistent ahrs_Q;     % estimate noise variance

ahrs_dt = 0.015625;
R = [ 1.3^2            % R is our measurement noise estimate
      1.3^2
      2.5^2 ];
%R = [ 0.0046^2       
%      0.0046^2
%      2.5^2 ];

if (status == AHRS_UNINIT)

  [ahrs_quat ...
   ahrs_biases ...
   ahrs_rates ...
   ahrs_P ...
   ahrs_Q ] = ahrs_init(gyro, accel, mag);
  
else
  [ahrs_quat ...
   ahrs_rates...
   ahrs_P] = ahrs_predict(ahrs_P, ahrs_Q, gyro, ahrs_quat, ahrs_biases,...
			  ahrs_dt);

  if (status == AHRS_STEP_PHI)
    measure = phi_of_accel(accel);
    estimate = phi_of_quat(ahrs_quat);
    C = get_dphi_dq(ahrs_quat);
  elseif (status == AHRS_STEP_THETA)
    measure = theta_of_accel(accel);
    estimate = theta_of_quat(ahrs_quat);
    C = get_dtheta_dq(ahrs_quat);
  elseif (status == AHRS_STEP_PSI)
    phi = phi_of_quat(ahrs_quat);
    theta = theta_of_quat(ahrs_quat);
    measure = psi_of_mag(mag, phi, theta);
    estimate = psi_of_quat(ahrs_quat);
    C = get_dpsi_dq(ahrs_quat);
  end;
  
  error = measure - estimate;
  [ahrs_quat   ...
   ahrs_biases ...
   ahrs_P] = ahrs_update(C, error, R(status), ahrs_P, ahrs_quat, ...
			 ahrs_biases);
end;

quat = ahrs_quat;
biases = ahrs_biases;


%
%
% Initialisation
%
%
function [quat, biases, rates, P, Q] = ahrs_init(gyro, accel, mag)
phi = phi_of_accel(accel);
theta = theta_of_accel(accel);
psi = psi_of_mag(mag, phi, theta);
quat = quat_of_eulers([phi, theta, psi]);
biases = gyro;
rates = [0 0 0]';
P = [ 1 0 0 0 0 0 0
      0 1 0 0 0 0 0
      0 0 1 0 0 0 0
      0 0 0 1 0 0 0
      0 0 0 0 0 0 0
      0 0 0 0 0 0 0
      0 0 0 0 0 0 0 ];

qg = 8e-03;
%qg = 1e-04;
Q = [ 0 0 0 0  0  0  0
      0 0 0 0  0  0  0
      0 0 0 0  0  0  0
      0 0 0 0  0  0  0
      0 0 0 0 qg  0  0
      0 0 0 0  0 qg  0
      0 0 0 0  0  0 qg ];


%
%
% Prediction
%
%
function [quat_out, rates_out, P_out] = ahrs_predict(P_in, Q_in, ...
						     gyro, quat_in,  biases, ...
						     dt)
rates_out = gyro - biases;
p = rates_out(1);
q = rates_out(2);
r = rates_out(3);

omega = 0.5 * [ 0 -p -q -r 
		p  0  r -q
		q -r  0  p
		r  q -p  0 ]; 


quat_dot = omega * quat_in;

quat_out = quat_in + quat_dot * dt;

quat_out = normalize_quat(quat_out);

% F is the Jacobian of Xdot with respect to the states
q0 = quat_out(1);
q1 = quat_out(2);
q2 = quat_out(3);
q3 = quat_out(4);


F =  0.5 * [ 0 -p -q -r  q1  q2  q3
	     p  0  r -q -q0  q3 -q2 
	     q -r  0  p -q3 -q0  q1
	     r  q -p  0  q2 -q1 -q0 
	     0  0  0  0  0   0   0 
	     0  0  0  0  0   0   0 
	     0  0  0  0  0   0   0 ];

P_dot = F * P_in + P_in * F' + Q_in;

P_out = P_in + P_dot * dt;

%
%
% Update
%
%
function [quat_out, biases_out, P_out] = ahrs_update(C, err, R, P_in, ...
						     quat_in, biases_in)
E = C * P_in * C' + R;

K = P_in * C' * inv(E);

P_out = P_in - K * C * P_in;

X = [quat_in' biases_in']';

X = X + K *err;

quat_out = [X(1) X(2) X(3) X(4)]';
biases_out = [X(5) X(6) X(7)]';
quat_out = normalize_quat(quat_out);


%
% Jacobian of the measurements to the system states.
%
function [C] = get_dphi_dq(quat)
dcm = dcm_of_quat(quat);
phi_err = 2 / (dcm(3,3)^2 + dcm(2,3)^2);
q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);
C = [ 
    (q1 * dcm(3,3))                     * phi_err
    (q0 * dcm(3,3) + 2 * q1 * dcm(2,3)) * phi_err
    (q3 * dcm(3,3) + 2 * q2 * dcm(2,3)) * phi_err
    (q2 * dcm(3,3))                     * phi_err 
    0
    0
    0 
    ]';

function [C] = get_dtheta_dq(quat)
dcm = dcm_of_quat(quat);
theta_err = 2 / sqrt(1 - dcm(1,3)^2);
q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);
C = [
     q2 * theta_err
    -q3 * theta_err
     q0 * theta_err
    -q1 * theta_err 
     0
     0
     0 
    ]';

function [C] = get_dpsi_dq(quat)
dcm = dcm_of_quat(quat);
psi_err = 2 / (dcm(1,1)^2 + dcm(1,2)^2);
q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);
C = [
    (q3 * dcm(1,1))                     * psi_err
    (q2 * dcm(1,1))                     * psi_err
    (q1 * dcm(1,1) + 2 * q2 * dcm(1,2)) * psi_err
    (q0 * dcm(1,1) + 2 * q3 * dcm(1,2)) * psi_err 
    0
    0
    0
    ]';




