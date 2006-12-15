clear

log = 'data/log_ahrs_bug';
%log = 'data/log_imu_still';
%log = 'data/log_imu_roll';
%log = 'data/log_ahrs_still';
%log = 'data/log_ahrs_roll';
%log = 'data/log_ahrs_yaw';
%log = 'data/log_ahrs_yaw_pitched';

[gyro, accel, mag] = read_imu_log(log);


tilt_status = 0;                      % uninit
[angle, bias] = tilt(tilt_status, gyro(:,1), accel(:,1));

%[n, m] = size(accel)
[n, m] = size(gyro)

tilt_status = 1;
for idx = 1:m
  [angle, bias] = tilt(tilt_status, gyro(:,idx), accel(:,idx));
  saved_t(idx) = idx;
  saved_theta(idx) = angle;
  saved_by(idx) = bias;
  theta_measure(idx) = theta_of_accel(accel(:,idx));

end;

subplot(3,1,1)
plot(saved_t, theta_measure, saved_t, saved_theta);
title('angle');
legend('measure', 'estimation');

subplot(3,1,2)
plot (saved_t, gyro(2,:));
title('rate');


subplot(3,1,3)
plot(saved_t, saved_by);
title('bias');


