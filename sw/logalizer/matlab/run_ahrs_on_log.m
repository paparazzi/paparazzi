clear

log = 'data/log_ahrs_bug';
%log = 'data/log_imu_still';
%log = 'data/log_imu_roll';
%log = 'data/log_ahrs_still';
%log = 'data/log_ahrs_roll';
%log = 'data/log_ahrs_yaw';
%log = 'data/log_ahrs_yaw_pitched';

[gyro, accel, mag] = read_imu_log(log);


ahrs_status = 0;                      % uninit
nb_init = 160;
m_gyro = [ mean(gyro(1, 1:nb_init))
	   mean(gyro(2, 1:nb_init))
	   mean(gyro(3, 1:nb_init)) ]
m_accel = [ mean(accel(1, 1:nb_init))
	    mean(accel(2, 1:nb_init))
	    mean(accel(3, 1:nb_init)) ]
m_mag = [ mean(mag(1, 1:nb_init))
	  mean(mag(2, 1:nb_init))
	  mean(mag(3, 1:nb_init)) ]
[quat, biases] = ahrs(ahrs_status, m_gyro, m_accel, m_mag);

sensor_length = [length(mag) length(accel) length(gyro)]
m = min(sensor_length)

for idx = 1:m
  
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

subplot(3,1,1)
plot(saved_t, saved_phi, saved_t, saved_theta, saved_t, saved_psi, ...
     saved_t, phi_measure, saved_t, theta_measure, saved_t, psi_measure);
title('eulers (matlab)');
legend('ahrs phi','ahrs theta','ahrs psi', 'measure phi', 'measure theta', ...
       'measure psi');

subplot(3,1,2)
plot (saved_t, gyro(1,:), saved_t, gyro(2,:), saved_t, gyro(3,:));
title('gyros');
legend('gyro x','gyro y','gyro z');

subplot(3,1,3)
plot(saved_t, saved_bx, saved_t, saved_by, saved_t, saved_bz);
title('biases (matlab)');
legend('bias x','bias y','bias z');


%[n, m] = size(ab_ahrs)
%for idx = 1:m
%  quat = [ab_ahrs(1,idx) ab_ahrs(2,idx) ab_ahrs(3,idx) ab_ahrs(4,idx)];
%  eulers = eulers_of_quat(quat);
%  phi_ab(idx) = eulers(1);
%  theta_ab(idx) = eulers(2);
%  psi_ab(idx) = eulers(3);
%  t_ab(idx) = idx;
%end;

%subplot(4,1,2)
%plot(t_ab, phi_ab, t_ab, theta_ab, t_ab, psi_ab);
%title('eulers (airborne)');

%subplot(4,1,4)
%plot(t_ab, ab_ahrs(5,:), t_ab, ab_ahrs(6,:), t_ab, ab_ahrs(7,:));
%title('biases (airborne)');

