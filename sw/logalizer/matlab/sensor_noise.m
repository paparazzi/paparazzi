%
%
%

clear

[gyro, accel, mag] = read_imu_log('data/log_ahrs_still');

[n m] = size(accel);

m_ax = mean(accel(1, :));
std_dev_ax = std(accel(1, :));
m_ay = mean(accel(2, :));
std_dev_ay = std(accel(2, :));
m_az = mean(accel(3, :));
std_dev_az = std(accel(3, :));

disp(sprintf('accel : mean %f %f %f  std dev %f %f %f',...
                 m_ax, m_ay, m_az, std_dev_ax, std_dev_ay, std_dev_az));

m_mx = mean(mag(1, :));
std_dev_mx = std(mag(1, :));
m_my = mean(mag(2, :));
std_dev_my = std(mag(2, :));
m_mz = mean(mag(3, :));
std_dev_mz = std(mag(3, :));

disp(sprintf('mag   : mean %f %f %f  std dev %f %f %f',...
                 m_mx, m_my, m_mz, std_dev_mx, std_dev_my, std_dev_mz));

m_gx = mean(gyro(1, :));
std_dev_gx = std(gyro(1, :));
m_gy = mean(gyro(2, :));
std_dev_gy = std(gyro(2, :));
m_gz = mean(gyro(3, :));
std_dev_gz = std(gyro(3, :));

disp(sprintf('gyro  : mean %f %f %f  std dev %f %f %f',...
                 m_gx, m_gy, m_gz, std_dev_gx, std_dev_gy, std_dev_gz));

for idx = 1:m
  phi(idx) = phi_of_accel(accel(:,idx));
  theta(idx) = theta_of_accel(accel(:,idx));
end;

m_phi = mean(phi);
m_theta = mean(theta);

std_dev_phi = std(phi);
std_dev_theta = std(theta);

subplot(3, 2, 1);
plot(phi);
title('phi (accel)');
subplot(3, 2, 2);
hist(phi);
title(sprintf('mean %f stddev %f', m_phi, std_dev_phi));

subplot(3, 2, 3);
plot(theta);
title('theta (accel)');
subplot(3, 2, 4);
hist(theta);
title(sprintf('mean %f stddev %f', m_theta, std_dev_theta));

[n m] = size(mag);
for idx = 1:m
  psi(idx) = psi_of_mag(mag(:,idx), m_phi, m_theta);
end;

m_psi = mean(psi);
std_dev_psi = std(psi);

subplot(3, 2, 5);
plot(psi);
title('psi (mag)');
subplot(3, 2, 6);
hist(psi);
title(sprintf('mean %f stddev %f', m_psi, std_dev_psi));

disp(sprintf('measures : mean %f %f %f  std dev %f %f %f',...
                 m_phi, m_theta, m_psi, ...
                 std_dev_phi, std_dev_theta, std_dev_phi));

