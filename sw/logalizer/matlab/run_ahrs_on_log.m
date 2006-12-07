clear

%fid = fopen('data/log_imu_still', 'r');
%fid = fopen('data/log_imu_roll', 'r');
%fid = fopen('data/log_ahrs_still', 'r');
%fid = fopen('data/log_ahrs_roll', 'r');
%fid = fopen('data/log_ahrs_yaw', 'r');
%fid = fopen('data/log_ahrs_yaw_pitched', 'r');
fid = fopen('data/log_ahrs_bug', 'r');

mag=[];
accel=[];
gyro=[];
ab_ahrs=[];

while 1
  tline = fgetl(fid);
  if ~ischar(tline),   break,   end
%  disp(tline)
  [A, count] = sscanf(tline, 'IMU_MAG %d %d %d');
  if (count == 3), mag = [mag A];, end;
  [A, count] = sscanf(tline, 'IMU_ACCEL %f %f %f');
  if (count == 3), accel = [accel A];, end;
  [A, count] = sscanf(tline, 'IMU_GYRO %f %f %f');
  if (count == 3), gyro = [gyro A];, end;
  [A, count] = sscanf(tline, 'AHRS_STATE %f %f %f %f %f %f %f');
  if (count == 7), ab_ahrs = [ab_ahrs A];, end;
end

%mag
%accel
%gyro
%ab_ahrs

%plot(mag(3,:))
%plot(gyro(3,:))
%plot(accel(3,:))

ahrs_status = 0;                      % uninit
[quat, biases] = ahrs(ahrs_status, gyro(:,1), accel(:,1), mag(:,1));

[n, m] = size(mag)
%[n, m] = size(accel)
%[n, m] = size(gyro)

for idx = 1:m-1
  
  ahrs_status = 1 + mod(idx, 3);
  [quat, biases] = ahrs(ahrs_status, gyro(:,idx), accel(:,idx), mag(:,idx));
  saved_t(idx) = idx;
  eulers = eulers_of_quat(quat);
  saved_phi(idx) = eulers(1);
  saved_theta(idx) = eulers(2);
  saved_psi(idx) = eulers(3);
  saved_bx(idx) = biases(1);
  saved_by(idx) = biases(2);
  saved_bz(idx) = biases(3);
  phi_measure(idx) = phi_of_accel(accel(:,idx));
  theta_measure(idx) = theta_of_accel(accel(:,idx));
  psi_measure(idx) = psi_of_mag(mag(:,idx), phi_measure(idx), theta_measure(idx));
    
end;

subplot(2,1,1)
plot(saved_t, saved_phi, saved_t, saved_theta, saved_t, saved_psi, saved_t, theta_measure, saved_t, psi_measure);
title('eulers (matlab)');

subplot(2,1,2)
plot(saved_t, saved_bx, saved_t, saved_by, saved_t, saved_bz);
title('biases (matlab)');


[n, m] = size(ab_ahrs)
for idx = 1:m
  quat = [ab_ahrs(1,idx) ab_ahrs(2,idx) ab_ahrs(3,idx) ab_ahrs(4,idx)];
  eulers = eulers_of_quat(quat);
  phi_ab(idx) = eulers(1);
  theta_ab(idx) = eulers(2);
  psi_ab(idx) = eulers(3);
  t_ab(idx) = idx;
end;

%subplot(4,1,2)
%plot(t_ab, phi_ab, t_ab, theta_ab, t_ab, psi_ab);
%title('eulers (airborne)');

%subplot(4,1,4)
%plot(t_ab, ab_ahrs(5,:), t_ab, ab_ahrs(6,:), t_ab, ab_ahrs(7,:));
%title('biases (airborne)');

