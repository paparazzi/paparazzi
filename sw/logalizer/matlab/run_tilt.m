clear

dt = 0.015625;

use_log = 0;

if (use_log)
  %log = 'data/log_ahrs_bug';
  log = 'data/log_ahrs_still';
  %log = 'data/log_ahrs_roll';
  %log = 'data/log_ahrs_yaw';
  %log = 'data/log_ahrs_yaw_pitched';
  [gyro, accel, mag] = read_imu_log(log);
  t = 0:dt:(length(gyro)-1)*dt;
else
  nb_samples = 3500;
  [t, rates, quat] = synth_data(dt, nb_samples);
  [gyro, accel, mag] =  synth_imu(rates, quat);
end

tilt_status = 0;                      % uninit
nb_init = 180;
m_gyro = [ mean(gyro(1, 1:nb_init))
	   mean(gyro(2, 1:nb_init))
	   mean(gyro(3, 1:nb_init)) ];
m_accel = [ mean(accel(1, 1:nb_init))
	    mean(accel(2, 1:nb_init))
	    mean(accel(3, 1:nb_init)) ];
%[angle, bias] = 
tilt(tilt_status, m_gyro, m_accel);

[n, m] = size(gyro);

for idx = 1:m
  if ( mod(idx, 10) == 0)
    tilt_status = 2;
  else
    tilt_status = 1;
  end
  [theta_est(idx), bias(idx)] = tilt(tilt_status, gyro(:,idx), accel(:,idx));
  theta_measure(idx) = theta_of_accel(accel(:,idx));
end;

subplot(3,1,1)
plot(t, theta_measure, t, theta_est, 'r');
title('angle');
legend('measure', 'estimation');

subplot(3,1,2)
plot (t, gyro(2,:));
title('rate');


avg_bias = mean(bias)
std_bias = std(bias)

subplot(3,1,3)
plot(t, bias, t, avg_bias * ones(size(bias)),...
     t, (avg_bias + std_bias) * ones(size(bias)),...
     t, (avg_bias - std_bias) * ones(size(bias)));
title('bias');


