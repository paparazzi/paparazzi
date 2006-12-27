
function [sys,x0,str,ts] = sfun_ahrs(t,x,u,flag)

AHRS_UNINIT     = 0;
AHRS_STEP_PHI   = 1;
AHRS_STEP_THETA = 2;
AHRS_STEP_PSI   = 3;

persistent ahrs_state;
persistent ahrs_quat;  % first four elements of our state
persistent ahrs_biases;% last three elements of our state
persistent ahrs_rates; % we get unbiased body rates as byproduct
persistent ahrs_P;     % error covariance matrix


ahrs_dt = 0.015625;

%R = [ 1.3^2            % R is our measurement noise estimate
%      1.3^2
%      2.5^2 ];

R = [ 4^2               % R is our measurement noise estimate
      4^2
      4^2 ];


%qg = 8e-03;
qg = 8e-3;
Q = [ 0 0 0 0  0  0  0
      0 0 0 0  0  0  0
      0 0 0 0  0  0  0
      0 0 0 0  0  0  0
      0 0 0 0 qg  0  0
      0 0 0 0  0 qg  0
      0 0 0 0  0  0 qg ];

%disp(sprintf('flag %d', flag));
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
 case 0,
  ahrs_state = 0;
  ahrs_quat = quat_of_eulers([0 0 0]');
  ahrs_biases = [0 0 0]';
  ahrs_rates = [0 0 0]';
  ahrs_init(1, [0 0 0]', [0 0 0], [0 0 0]);
  [sys,x0,str,ts]=mdlInitializeSizes(ahrs_dt);
  
  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
 case 2,
  gyro  = u(1:3);
  mag   = u(4:6);
  accel = u(7:9);
  if (ahrs_state == AHRS_UNINIT)
    [filter_initialised, ahrs_quat ahrs_biases ahrs_rates ahrs_P] = ...
	ahrs_init(0, gyro, accel, mag);
    if( filter_initialised ) ahrs_state = AHRS_STEP_PHI;, end;
  else
    [ahrs_quat ahrs_rates ahrs_P] = ...
	ahrs_predict(ahrs_P, Q, gyro, ahrs_quat, ahrs_biases,...
		     ahrs_dt);
    switch ahrs_state,
     case AHRS_STEP_PHI,
      measure = phi_of_accel(accel);
      estimate = phi_of_quat(ahrs_quat);
      wrap = pi;
      H = get_dphi_dq(ahrs_quat);
     case AHRS_STEP_THETA,
      measure = theta_of_accel(accel);
      estimate = theta_of_quat(ahrs_quat);
      wrap = pi/2;
      H = get_dtheta_dq(ahrs_quat);
     case AHRS_STEP_PSI,
      phi = phi_of_quat(ahrs_quat);
      theta = theta_of_quat(ahrs_quat);
      measure = psi_of_mag(mag, phi, theta);
      estimate = psi_of_quat(ahrs_quat);
      wrap = pi;
      H = get_dpsi_dq(ahrs_quat);
    end;
    error = get_error(measure, estimate, wrap);
    [ahrs_quat ahrs_biases ahrs_P] = ...
	ahrs_update(H, error, R(ahrs_state), ahrs_P, ahrs_quat, ahrs_biases);
    ahrs_state = ahrs_state+1;
    if (ahrs_state > AHRS_STEP_PSI), ahrs_state = AHRS_STEP_PHI;, end; 
  end;
  sys = [];
  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
   eulers = eulers_of_quat(ahrs_quat);
   sys = [eulers(1) eulers(2) eulers(3) ahrs_rates(1) ahrs_rates(2) ...
	  ahrs_rates(3) ahrs_biases(1) ahrs_biases(2) ahrs_biases(3)];
  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys = [];
  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end  
   

%
% begin mdlInitializeSizes 
%
function [sys,x0,str,ts]=mdlInitializeSizes (period)   
sizes = simsizes();

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 9;
sizes.NumInputs      = 9;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);

x0  = [];

str = [];

ts  = [period 0];

% end mdlInitializeSizes 

%
%
% Initialisation
%
%
function [filter_initialised, quat, biases, rates, P] = ...
    ahrs_init(reset, gyro, accel, mag)
persistent saved_gyro;
persistent saved_accel;
persistent saved_mag;
persistent nb_init;

if (reset)
  nb_init = 0;
  saved_gyro = [];
  saved_accel = [];
  saved_mag = [];
  return;
end;

nb_init = nb_init+1;
saved_gyro(:, nb_init) = gyro;
saved_accel(:, nb_init) = accel;
saved_mag(:, nb_init) = mag;

if (nb_init < 50)
  quat = quat_of_eulers([0 0 0]');
  biases = [0 0 0]';
  filter_initialised = 0;
else
  mean_gyro = average_vector(saved_gyro);
  mean_accel = average_vector(saved_accel);
  mean_mag = average_vector(saved_mag);
  phi = phi_of_accel(mean_accel);
  theta = theta_of_accel(mean_accel);
  psi = psi_of_mag(mean_mag, phi, theta);
  quat = quat_of_eulers([phi, theta, psi]);
  biases = mean_gyro;
  mgd = mean_gyro * 180 / pi;
  atd = [phi, theta, psi] * 180 / pi;
  disp('init done');
  disp(sprintf('initial biases %f %f %f',mgd(1), mgd(2), mgd(3))); 
  disp(sprintf('initial attitude %f %f %f',atd(1), atd(2), atd(3))); 
  filter_initialised = 1;
end;
  rates = [0 0 0];
  P = [ 1 0 0 0 0 0 0
	0 1 0 0 0 0 0
	0 0 1 0 0 0 0
	0 0 0 1 0 0 0
	0 0 0 0 0 0 0
	0 0 0 0 0 0 0
	0 0 0 0 0 0 0 ];

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
function [quat_out, biases_out, P_out] = ahrs_update(H, err, R, P_in, ...
						     quat_in, biases_in)
E = H * P_in * H' + R;

K = P_in * H' * inv(E);

P_out = P_in - K * H * P_in;

X = [quat_in' biases_in']';

X = X + K *err;

quat_out = [X(1) X(2) X(3) X(4)]';
biases_out = [X(5) X(6) X(7)]';
quat_out = normalize_quat(quat_out);


%
% Jacobian of the measurements to the system states.
%
function [H] = get_dphi_dq(quat)
dcm = dcm_of_quat(quat);
phi_err = 2 / (dcm(3,3)^2 + dcm(2,3)^2);
q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);
H = [ 
    (q1 * dcm(3,3))                     * phi_err
    (q0 * dcm(3,3) + 2 * q1 * dcm(2,3)) * phi_err
    (q3 * dcm(3,3) + 2 * q2 * dcm(2,3)) * phi_err
    (q2 * dcm(3,3))                     * phi_err 
    0
    0
    0 
    ]';

function [H] = get_dtheta_dq(quat)
dcm = dcm_of_quat(quat);
theta_err = 2 / sqrt(1 - dcm(1,3)^2);
q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);
H = [
     q2 * theta_err
    -q3 * theta_err
     q0 * theta_err
    -q1 * theta_err 
     0
     0
     0 
    ]';

function [H] = get_dpsi_dq(quat)
dcm = dcm_of_quat(quat);
psi_err = 2 / (dcm(1,1)^2 + dcm(1,2)^2);
q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);
H = [
    (q3 * dcm(1,1))                     * psi_err
    (q2 * dcm(1,1))                     * psi_err
    (q1 * dcm(1,1) + 2 * q2 * dcm(1,2)) * psi_err
    (q0 * dcm(1,1) + 2 * q3 * dcm(1,2)) * psi_err 
    0
    0
    0
    ]';

function [err] = get_error(measure, estimate, wrap)
err = measure - estimate;
if (err > wrap), err = err - 2*wrap, end;
if (err < -wrap), err = err + 2*wrap, end;

function [avg] = average_vector(vector)
avg = [ mean(vector(1, 1:length(vector)))
	mean(vector(2, 1:length(vector)))
	mean(vector(3, 1:length(vector))) ];