clear

dt = 0.015625;

use_log = 1;

if (use_log)
  log = 'data/log_ahrs_bug';
  %log = 'data/log_ahrs_still';
  %log = 'data/log_ahrs_roll';
  %log = 'data/log_ahrs_yaw';
  %log = 'data/log_ahrs_yaw_pitched';
  [gyro, accel, mag] = read_imu_log(log);
  t = 0:dt:(length(gyro)-1)*dt;
else
  nb_samples = 1500;
  [t, rates, quat] = synth_data(dt, nb_samples);
  [gyro, accel, mag] =  synth_imu(rates, quat);
end

% initialisation
tilt_status = 0;
nb_init = 180;
m_gyro = [ mean(gyro(1, 1:nb_init))
	   mean(gyro(2, 1:nb_init))
	   mean(gyro(3, 1:nb_init)) ];
m_accel = [ mean(accel(1, 1:nb_init))
	    mean(accel(2, 1:nb_init))
	    mean(accel(3, 1:nb_init)) ];

tilt(tilt_status, m_gyro, m_accel);

for idx = 1:length(gyro)
  if ( mod(idx, 10) == 0)
    tilt_status = 2;
  else
    tilt_status = 1;
  end
  [theta_est(idx), bias(idx), rate_est(idx), cov(:,:,idx)] = tilt(tilt_status, gyro(:,idx), accel(:,idx));
  theta_measure(idx) = theta_of_accel(accel(:,idx));
%  theta_measure(idx) = phi_of_accel(accel(:,idx));
end;

subplot(4,1,1)
plot(t, theta_measure*180/pi, t, theta_est*180/pi, 'r');
title('angle');
legend('measure', 'estimation');
xlabel('time in s');
ylabel('angle in degres');

subplot(4,1,2)
plot (t, gyro(2,:)*180/pi, t, rate_est*180/pi);
title('rate');
legend('measure', 'estimation');
ylabel('rate in degres/s');

avg_bias = mean(bias)*180/pi
std_bias = std(bias)*180/pi

subplot(4,1,3)
plot(t, bias*180/pi, ...
     t, avg_bias * ones(size(bias)),...
     t, (avg_bias + std_bias) * ones(size(bias)),...
     t, (avg_bias - std_bias) * ones(size(bias)));
title('bias');
ylabel('bias in degres/s');

for idx=1:length(gyro)
  P00(idx) = cov(1,1,idx);
  P01(idx) = cov(1,2,idx);
  P11(idx) = cov(2,2,idx);
end;

subplot(4,1,4)
plot(t, P00, t, P01, t, P11)
legend('P00', 'P01', 'P11');
title('error covariance');