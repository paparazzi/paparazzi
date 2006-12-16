function [angle, bias, rate] = tilt(status, gyro, accel)


TILT_UNINIT     = 0;
TILT_PREDICT    = 1;
TILT_UPDATE     = 2;

persistent tilt_angle; % our state
persistent tilt_bias;  %
persistent tilt_rate;  % unbiased rate
persistent tilt_P;     % covariance matrix

tilt_dt = 0.015625;    % time step
tilt_R =  0.3;         % measurement covariance noise
                       % means we expect a 0.3 rad jitter from the
                       % accelerometer

		       
		       
if (status == TILT_UNINIT)
  [tilt_angle, tilt_bias, tilt_rate, tilt_P] = tilt_init(gyro, accel);
else
  [tilt_angle, tilt_rate, tilt_P] = tilt_predict(gyro, tilt_P, ...
						 tilt_angle, tilt_bias, ...
						 tilt_dt);
  if (status == TILT_UPDATE)
    [tilt_angle, tilt_bias, tilt_P] = tilt_update(accel, tilt_R, tilt_P, ...
						  tilt_angle, ...
						  tilt_bias);
  end
end		       

angle = tilt_angle;
bias = tilt_bias;
rate = tilt_rate;

		       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Initialisation
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [angle, bias, rate, P] = tilt_init(gyro, accel)
angle = theta_of_accel(accel);

bias = gyro(2);

rate = 0;

%P = [ 1 0
%      0 1 ];
P = [ 1 0
      0 0 ];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Prediction
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [angle_out, rate_out, P_out] = tilt_predict(gyro, P_in, ...
						     angle_in,  bias, ...
						     dt)
rate_out = gyro(2) - bias;

% update state ( X += Xdot * dt )
angle_out = angle_in + rate_out * dt;

% update covariance ( Pdot = A*P + P*A' + Q )
%                   ( P += Pdot * dt        ) 
%
% A is the Jacobian of Xdot with respect to the states:
%
%      A = [ d(angle_dot)/d(angle)     d(angle_dot)/d(gyro_bias) ]
%          [ d(gyro_bias_dot)/d(angle) d(gyro_bias_dot)/d(gyro_bias) ]
%

A = [ 0 -1            % jacobian of state dot wrt state
      0  0 ];

Q = [ 0  0            % process covariance noise
      0  8e-3 ];

Pdot = A * P_in + P_in * A' + Q;

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
err = measure_angle - angle_in;

C = [ 1 0 ];

E = C * P_in * C' + R;

K = P_in * C' * inv(E);

P_out = P_in - K * C * P_in;

X = [ angle_in
      bias_in  ];

X = X + K *err;
      
angle_out = X(1);
bias_out = X(2);

