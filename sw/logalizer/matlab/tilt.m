%
% this is a 2 states kalman filter used to fuse the readings of a
% two axis accelerometer and one axis gyro.
% The filter estimates the angle and the gyro bias.
%
%
function [angle, bias, rate, cov] = tilt(status, gyro, accel)


TILT_UNINIT     = 0;
TILT_PREDICT    = 1;
TILT_UPDATE     = 2;

persistent tilt_angle; % our state
persistent tilt_bias;  %
persistent tilt_rate;  % unbiased rate
persistent tilt_P;     % covariance matrix

tilt_dt = 0.015625;    % prediction time step
tilt_R =  0.3;         % measurement covariance noise
                       % means we expect a 0.3 rad jitter from the
                       % accelerometer
		       
if (status == TILT_UNINIT)
  [tilt_angle, tilt_bias, tilt_rate, tilt_P] = tilt_init(gyro, accel);
else
  [tilt_angle, tilt_rate, tilt_P] = ...
      tilt_predict(gyro, tilt_P, ...
		   tilt_angle, tilt_bias, tilt_rate, ...
		   tilt_dt);
  if (status == TILT_UPDATE)
    [tilt_angle, tilt_bias, tilt_P] = tilt_update(accel, tilt_R, tilt_P, ...
						  tilt_angle, tilt_bias);
  end
end		       

angle = tilt_angle;
bias = tilt_bias;
rate = tilt_rate;
cov = tilt_P;
		       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Initialisation
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [angle, bias, rate, P] = tilt_init(gyro, accel)
angle = theta_of_accel(accel);
%angle = phi_of_accel(accel);

bias = gyro(2);

rate = 0;

P = [ 1 0
      0 0 ];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Prediction
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [angle_out, rate_out, P_out] = tilt_predict(gyro, P_in, ...
						     angle_in,  bias, rate,...
						     dt)
rate_out = gyro(2) - bias;
%rate_out = gyro(1) - bias;

% update state ( X += Xdot * dt )
angle_out = angle_in + (rate + rate_out) / 2 * dt;

% update covariance ( Pdot = F*P + P*F' + Q )
%                   ( P += Pdot * dt        ) 
%
% F is the Jacobian of Xdot with respect to the states:
%
%      F = [ d(angle_dot)/d(angle)     d(angle_dot)/d(gyro_bias) ]
%          [ d(gyro_bias_dot)/d(angle) d(gyro_bias_dot)/d(gyro_bias) ]
%

F = [ 0 -1            % jacobian of state dot wrt state
      0  0 ];

Q = [ 0  0            % process covariance noise
      0  8e-3 ];

Pdot = F * P_in + P_in * F' + Q;

P_out = P_in + Pdot * dt;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Update
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [angle_out, bias_out, P_out] = tilt_update(accel, R, P_in, ...
						    angle_in, ...
						    bias_in)

measure_angle = theta_of_accel(accel);
%measure_angle = phi_of_accel(accel);
err = measure_angle - angle_in;

H = [ 1 0 ];

E = H * P_in * H' + R;

K = P_in * H' * inv(E);

P_out = P_in - K * H * P_in;

X = [ angle_in
      bias_in  ];

X = X + K *err;
      
angle_out = X(1);
bias_out = X(2);

