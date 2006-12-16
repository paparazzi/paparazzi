clear

dt = 0.015625;
nb_samples = 1500;

%
% generate a set of flight parameters
%
[t, rates, quat] = synth_data(dt, nb_samples);

subplot(3, 2, 1);
plot(t, rates(1,:), t, rates(2,:), t, rates(3,:));
title('synthetic rates');
legend('p', 'q', 'r');

for idx=1:nb_samples
  eulers(:, idx) = eulers_of_quat(quat(:, idx));
end

subplot(3, 2, 2);
plot(t, eulers(1,:), t, eulers(2,:), t, eulers(3,:));
title('synthetic eulers');
legend('phi', 'theta', 'psi');

%
% generate a set of IMU sensors readings
%
[gyro, accel, mag] =  synth_imu(rates, quat);

subplot(3, 2, 3);
plot(t, gyro(1,:), t, gyro(2,:), t, gyro(3,:));
title('synthetic gyro');
legend('p', 'q', 'r');

for idx=1:nb_samples
  ahrs_measure(1, idx) = phi_of_accel(accel(:, idx));
  ahrs_measure(2, idx) = theta_of_accel(accel(:, idx));
  ahrs_measure(3, idx) = psi_of_mag(mag(:, idx), ...
				    ahrs_measure(1, idx), ...
				    ahrs_measure(2, idx));
end

subplot(3, 2, 4);
plot(t, ahrs_measure(1,:), t, ahrs_measure(2,:), t, ahrs_measure(3,:));
title('synthetic eulers measures');
legend('phi', 'theta', 'psi');


%
% run ahrs
%

% initialisation
ahrs_status = 0;
nb_init = 140;
m_gyro = [ mean(gyro(1, 1:nb_init))
	   mean(gyro(2, 1:nb_init))
	   mean(gyro(3, 1:nb_init)) ]
m_accel = [ mean(accel(1, 1:nb_init))
	    mean(accel(2, 1:nb_init))
	    mean(accel(3, 1:nb_init)) ]
m_mag = [ mean(mag(1, 1:nb_init))
	  mean(mag(2, 1:nb_init))
	  mean(mag(3, 1:nb_init)) ]
%[ahrs_quat(:, 1), ahrs_biases(:, 1)] = 
ahrs(ahrs_status, m_gyro, m_accel, m_mag);

for idx = 1:nb_samples
  ahrs_status = 1 + mod(idx, 3);
  [ahrs_quat(:, idx), ahrs_biases(:, idx)] = ahrs(ahrs_status, ...
						  gyro(:, idx), ...
						  accel(:, idx), ... 
						  mag(:, idx));
  ahrs_eulers(:,idx) = eulers_of_quat(ahrs_quat(:, idx));
end


subplot(3, 2, 5);
plot(t, ahrs_biases(1,:), t, ahrs_biases(2,:), t, ahrs_biases(3,:));
title('ahrs estimate biases');
legend('bp', 'bq', 'br');

subplot(3, 2, 6);
plot(t, ahrs_eulers(1,:), t, ahrs_eulers(2,:), t, ahrs_eulers(3,:));
title('ahrs estimate eulers');
legend('phi', 'theta', 'psi');

