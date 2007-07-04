
function [eulers, biases] = eulers_ahrs(status, gyro, accel, mag, dt)

AHRS_UNINIT       = 0;
AHRS_PREDICT      = 1;
AHRS_UPDATE_PHI   = 2;
AHRS_UPDATE_THETA = 3;
AHRS_UPDATE_PSI   = 4;

persistent ahrs_eulers;
persistent ahrs_biases;
persistent ahrs_rates;

persistent ahrs_P;



if (status == AHRS_UNINIT)
  [ahrs_eulers ahrs_biases ahrs_rates ahrs_P] = ahrs_init(gyro, accel, mag);
else
  [ahrs_eulers, ahrs_rates, ahrs_P] = ahrs_predict(gyro, ahrs_P, ...
						   ahrs_eulers, ...
						   ahrs_rates, ahrs_biases, dt);
  if (status >= AHRS_UPDATE_PHI)
    [ahrs_eulers, ahrs_biases, ahrs_P] = ahrs_update(status, accel, mag, ...
						     ahrs_P, ahrs_eulers, ahrs_biases);

  
  end
end

eulers = ahrs_eulers;
biases = ahrs_biases;



%
%
% Initialisation
%
%
function [eulers, biases, rates, P] = ahrs_init(gyro, accel, mag)

eulers(1, 1) = phi_of_accel(accel);
eulers(2, 1) = theta_of_accel(accel);
eulers(3, 1) = psi_of_mag(mag, eulers(1), eulers(2));

biases = gyro;

rates = [0 0 0]';

P = [ 1 0 0 0 0 0
      0 1 0 0 0 0
      0 0 1 0 0 0
      0 0 0 0 0 0
      0 0 0 0 0 0
      0 0 0 0 0 0 ];

%
%
% Prediction
%
%
function [eulers_out, rates_out, P_out] = ahrs_predict(gyro, P_in, ...
						       eulers_in, rates_in, biases, ...
						       dt)
rates_out = gyro - biases;

phi   = eulers_in(1);
theta = eulers_in(2);
psi   = eulers_in(3);
p = rates_out(1);
q = rates_out(2);
r = rates_out(3);

rted = [ 1 sin(phi)*tan(theta)  cos(phi)*tan(theta)
	 0 cos(phi)            -sin(phi)
	 0 sin(phi)/cos(theta)  cos(phi)/cos(theta) ];

eulers_dot = rted * (rates_out + rates_in)/2;

eulers_out = eulers_in + eulers_dot * dt;


d_phidot_d_state = [ cos(phi)*tan(theta)*q - sin(phi)*tan(theta)*r 
                     1/(1+theta^2) * (sin(phi)*q+cos(phi)*r)
		     0
		    -1
		    -sin(phi)*tan(theta)
		    -cos(phi)*tan(theta) ]';

d_thetadot_d_state = [ -sin(phi)*q - cos(phi)*r
		        0
		        0
		        0
		       -cos(phi)
		        sin(phi) ]';
    
d_psidot_d_state = [ cos(phi)/cos(theta)*q - sin(phi)/cos(theta)*r
		     sin(theta)/cos(theta)^2*(sin(phi)*q+cos(phi)*r)
		     0
		     0
		     -sin(phi)/cos(theta)
		     -cos(phi)/cos(theta)
		   ]';

% jacobian of state_dot wrt state
F = [ d_phidot_d_state
      d_thetadot_d_state
      d_psidot_d_state
      0 0 0 0 0 0
      0 0 0 0 0 0
      0 0 0 0 0 0
    ];

% estimate noise covariance
Q = 8e-3 * [ 0 0 0 0 0 0
	     0 0 0 0 0 0
	     0 0 0 0 0 0
	     0 0 0 1 0 0
	     0 0 0 0 1 0
	     0 0 0 0 0 1 ];

P_dot = F * P_in + P_in * F' + Q;

P_out = P_in + P_dot * dt;


%
%
% Update
%
%
function [eulers_out, biases_out, P_out] = ahrs_update(status, accel, mag, ...
						       P_in, eulers_in, biases_in)

if (status == 2)%AHRS_UPDATE_PHI)
  measure = phi_of_accel(accel);
  estimate = eulers_in(1);
  H = [ 1 0 0 0 0 0 ];
  R = 1.3^2;
elseif (status == 3)%AHRS_UPDATE_THETA)
  measure = theta_of_accel(accel);
  estimate = eulers_in(2);
  H = [ 0 1 0 0 0 0 ];
  R = 1.3^2;
elseif (status == 4)%AHRS_UPDATE_PSI)
  measure = psi_of_mag(mag, eulers_in(1), eulers_in(2));
  estimate = eulers_in(3);
  H = [ 0 0 1 0 0 0 ];
  R = 2.5^2;
end

%R = [ 1.3^2            % R is our measurement noise estimate
%      1.3^2
%      2.5^2 ];

E = H * P_in * H' + R;

K = P_in * H' * inv(E);

P_out = P_in - K * H * P_in;

X = [eulers_in' biases_in']';

error = measure - estimate; 

X = X + K * error;

eulers_out = [X(1) X(2) X(3)]';

biases_out = [X(4) X(5) X(6)]';
