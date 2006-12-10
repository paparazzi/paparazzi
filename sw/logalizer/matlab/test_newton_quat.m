clear

g_ned = [ 0 
	  0 
	  258.3275];

h_ned = [ 166.8120 
	  0.0 
	  203.3070];

[gyro, accel, mag] = read_imu_log('data/log_ahrs_bug');

ahrs_status = 0;                      % uninit
[ahrs_quat, ahrs_biases] = ahrs(ahrs_status, gyro(:,1), accel(:,1), mag(:,1));
est_quat = quat_of_eulers([ 0 0 0 ]);

t=1:length(accel);

for idx=t
  g_bod = accel(:, idx);
  h_bod = mag(:, idx);

  [est_quat, norm_err, nb_iter] = newton_quat(est_quat, 20, 40, g_ned, h_ned, g_bod, h_bod);

  eulers = eulers_of_quat(est_quat);
  n_phi(idx) = eulers(1);
  n_theta(idx) = eulers(2);
  n_psi(idx) = eulers(3);
  n_error(idx) = norm_err;
  n_iter(idx) = nb_iter;
  
  
  ahrs_status = 1 + mod(idx, 3);
  [ahrs_quat, ahrs_biases] = ahrs(ahrs_status, gyro(:,idx), accel(:,idx), ...
				  mag(:,idx));
  eulers = eulers_of_quat(ahrs_quat);
  a_phi(idx) = eulers(1);
  a_theta(idx) = eulers(2);
  a_psi(idx) = eulers(3);
  
  m_phi(idx) =  phi_of_accel(accel(:,idx));
  m_theta(idx) =  theta_of_accel(accel(:,idx));
  m_psi(idx) = psi_of_mag(mag(:,idx), m_phi(idx), m_theta(idx));
  
end;

subplot(5, 1, 1);
plot(t, n_phi, t, a_phi, t, m_phi);
title('phi');

subplot(5, 1, 2);
plot(t, n_theta, t, a_theta, t, m_theta);
title('theta');

subplot(5, 1, 3);
plot(t, n_psi, t, a_psi, t, m_psi);
title('psi');

subplot(5, 1, 4);
plot(n_error);
title('error');

subplot(5, 1, 5);
plot(n_iter);
title('iterations');




























% looking for a quaternion n = (q0 q1 q2 q3) that would satisfy
% [accel, mag] = n [g_ned, h_ned] n*
%
% n = [ q0 q1 q2 q3 ];
%
% R = dcm_of_quat(n)';
%
% R = [ 
%  q0^2 + q1^2 - q2^2 - q3^2    2 * (q1*q2 - q0*q3)          2 * (q1*q3 + q0*q2)
%  2 * (q1*q2 + q0*q3)          q0^2 - q1^2 + q2^2 - q3^2    2 * (q2*q3 - q0*q1)
%  2 * (q1*q3 - q0*q2)          2 * (q2*q3 + q0*q1)          q0^2 - q1^2 - q2^2 + q3^2
% ];
%
%
% M = [ R 0
%       0 R ];
%  
%
% [accel_ned mag_ned]' = M * [accel_bod mag_bod]';
%
%
% 
% dR/dq0 = 2 * [  q0 -q3  q2
%                 q3  q0 -q1
%                -q2  q1  q0 ]; 
%
% dR/dq1 = 2 * [  q1  q2  q3
%                 q2 -q1 -q0
%                 q3  q0 -q1 ]; 
%
% dR/dq2 = 2 * [ -q2  q1  q0
%                 q1  q2  q3
%                -q0  q3 -q2 ]; 
%
% dR/dq3 = 2 * [ -q3 -q0  q1
%                 q0 -q3  q2
%                 q1  q2  q3 ];
%
%
%
%
% dM/dq0 = 2 * [  q0 -q3  q2   0   0   0
%                 q3  q0 -q1   0   0   0
%                -q2  q1  q0   0   0   0
%                  0   0   0  q0  -q3  q2
%                  0   0   0  q3   q0 -q1
%                  0   0   0 -q2   q1  q0 ];  
%
% 
%
% dM/dq1 = 2 * [  q1  q2  q3   0   0   0
%                 q2 -q1 -q0   0   0   0
%                 q3  q0 -q1   0   0   0
%                  0   0   0  q1  q2  q3
%                  0   0   0  q2 -q1 -q0
%                  0   0   0  q3  q0 -q1 ];
%
% dM/dq2 = 2 * [ -q2  q1  q0   0   0   0
%                 q1  q2  q3   0   0   0
%                -q0  q3 -q2   0   0   0
%                  0   0   0 -q2  q1  q0
%                  0   0   0  q1  q2  q3
%                  0   0   0 -q0  q3 -q2 ]; 
%
% dM/dq3 = 2 * [ -q3 -q0  q1   0   0   0
%                 q0 -q3  q2   0   0   0
%                 q1  q2  q3   0   0   0
%                  0   0   0 -q3 -q0  q1
%                  0   0   0  q0 -q3  q2
%                  0   0   0  q1  q2  q3 ];
% J' = 2 * [
%  q0  q3 -q2   0   0   0
% -q3  q0 -q1   0   0   0 
%  q2 -q1  q0   0   0   0
%   0   0   0  q0  q3 -q2
%   0   0   0 -q3  q0  q1 
%   0   0   0  q2 -q1  q0
%  q1  q2  q3   0   0   0
%  q2 -q1  q0   0   0   0
%  q3 -q0 -q1   0   0   0
%   0   0   0  q1  q2  q3 
%   0   0   0  q2 -q1  q0
%   0   0   0  q3 -q0 -q1
% -q2  q1 -q0   0   0   0
%  q1  q2  q3   0   0   0
%  q0  q3 -q2   0   0   0
%   0   0   0  -q2 q1 -q0
%   0   0   0   q1 q2  q3
%   0   0   0   q0 q3 -q2
% -q3  q0  q1   0   0   0
% -q0 -q3  q2   0   0   0
%  q1  q2  q3   0   0   0
%   0   0   0  -q3  q0  q1
%   0   0   0  -q0 -q3  q2
%   0   0   0   q1  q2  q3 
% ];
%
% J = 2 * [
%  q0 -q3  q2   0   0   0   q1  q2  q3  0   0   0  -q2  q1  q0   0   0   0  -q3 -q0  q1   0   0   0  
%  q3  q0 -q1   0   0   0   q2 -q1 -q0  0   0   0   q1  q2  q3   0   0   0   q0 -q3  q2   0   0   0
% -q2  q1  q0   0   0   0   q3  q0 -q1  0   0   0  -q0  q3 -q2   0   0   0   q1  q2  q3   0   0   0 
%   0   0   0   q0 -q3  q2  0   0   0  q1  q2  q3    0   0   0 -q2  q1  q0    0   0   0 -q3 -q0  q1
%   0   0   0   q3  q0 -q1  0   0   0  q2 -q1 -q0    0   0   0  q1  q2  q3    0   0   0  q0 -q3  q2
%   0   0   0  -q2  q1  q0  0   0   0  q3  q0 -q1    0   0   0 -q0  q3 -q2    0   0   0  q1  q2  q3
% ];
%
%
%
%
% J' * J = [
% dM/dq0*dM/dq0 dM/dq0*dM/dq1 dM/dq0*dM/dq2 dM/dq0*dM/dq3
% dM/dq1*dM/dq0 dM/dq1*dM/dq1 dM/dq1*dM/dq2 dM/dq1*dM/dq3
% dM/dq2*dM/dq0 dM/dq2*dM/dq1 dM/dq2*dM/dq2 dM/dq2*dM/dq3
% dM/dq3*dM/dq0 dM/dq3*dM/dq1 dM/dq3*dM/dq2 dM/dq3*dM/dq3
% ];
%
%
% err = [ g_ned    - M * [g_bod
%         h_ned ]         h_bod ];
%
%  from ( http://en.wikipedia.org/wiki/Gauss-Newton_algorithm )
%
% J' * J * delta = -J' * err;
%
%
%