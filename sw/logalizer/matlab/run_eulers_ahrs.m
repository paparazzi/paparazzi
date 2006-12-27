clear

dt = 0.015625;

log = 'data/log_ahrs_bug';
%log = 'data/log_ahrs_still';
%log = 'data/log_ahrs_roll';
%log = 'data/log_ahrs_yaw';
%log = 'data/log_ahrs_yaw_pitched';

[gyro, accel, mag] = read_imu_log(log);
t = 0:dt:(length(gyro)-1)*dt;

ahrs_status = 0;                      % uninit
nb_init = 100;
m_gyro =  [ mean(gyro(1, 1:nb_init))
	    mean(gyro(2, 1:nb_init))
	    mean(gyro(3, 1:nb_init)) ];
m_accel = [ mean(accel(1, 1:nb_init))
	    mean(accel(2, 1:nb_init))
	    mean(accel(3, 1:nb_init)) ];
m_mag =   [ mean(mag(1, 1:nb_init))
	    mean(mag(2, 1:nb_init))
	    mean(mag(3, 1:nb_init)) ];
[eulers, biases] = eulers_ahrs(ahrs_status, m_gyro, m_accel, m_mag, ...
			       dt);

for idx = 1:length(gyro)
  ahrs_status = 2 + mod(idx, 3);
  %ahrs_status = 1;
  %  if (mod(idx,10) == 0), ahrs_status = 2, end;
  [eulers_est(:, idx), biases_est(:, idx)] = ...
      eulers_ahrs(ahrs_status, gyro(:,idx), accel(:,idx), mag(:,idx), ...
		  dt);
  eulers_mea(1, idx) = phi_of_accel(accel(:,idx));
  eulers_mea(2, idx) = theta_of_accel(accel(:,idx));
  eulers_mea(3, idx) = psi_of_mag(mag(:,idx), eulers_mea(1, idx), eulers_mea(2, idx));
  
end

subplot(3,1,1)
plot(t, eulers_est(1,:), t, eulers_est(2,:), t, eulers_est(3,:), ...
     t, eulers_mea(1,:), t, eulers_mea(2,:), t, eulers_mea(3,:));
title('eulers');
legend('phi estimation', 'theta estimation', 'psi estimation',...
       'phi measure', 'theta measure', 'psi measure');

subplot(3,1,2)
plot (t, gyro(1,:), t, gyro(2,:), t, gyro(3,:));
title('gyros');
legend('gyro x','gyro y','gyro z');

%avg_bias = mean(biases_est)
%std_bias = std(biases_est)

subplot(3,1,3)
plot(t, biases_est(1,:), t, biases_est(2,:), t, biases_est(3,:));
%avg_bias * ones(size(biases_est)),...
%     t, (avg_bias + std_bias) * ones(size(biases_est)),...
%     t, (avg_bias - std_bias) * ones(size(biases_est)));
title('bias');
legend('bias p','bias q','bias r');