clear

[gyro, accel, mag] = read_imu_log('data/log_ahrs_still');

t=1:length(accel);

mean_accel_raw = mean(accel')
mean_accel_norm = norm(mean_accel_raw)

accel_cal = accel / mean_accel_norm * 9.81;

mean_phi_accel = phi_of_accel(mean_accel_raw)
mean_theta_accel = theta_of_accel(mean_accel_raw)



mean_mag_raw = mean(mag')

mean_psi_mag = psi_of_mag(mean_mag_raw, mean_phi_accel, mean_theta_accel)

mean_dcm = dcm_of_eulers([mean_phi_accel, mean_theta_accel, mean_psi_mag])

accel_ned = mean_dcm' * mean_accel_raw'
mag_ned = mean_dcm' * mean_mag_raw'

subplot(2, 1, 1);
plot(accel_cal');



subplot(2, 1, 2);
plot(mag');
