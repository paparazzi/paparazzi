
%
%
%

clear
fid = fopen('data/log_ahrs_still', 'r');

mag=[];
accel=[];

while 1
  tline = fgetl(fid);
  if ~ischar(tline),   break,   end
  [A, count] = sscanf(tline, 'IMU_MAG %d %d %d');
  if (count == 3), mag = [mag A];, end;
  [A, count] = sscanf(tline, 'IMU_ACCEL %f %f %f');
  if (count == 3), accel = [accel A];, end;
end


%m_accel = mean(accel, 2)
%plot(accel(3,:))
%hist(accel(3,:))

[n m] = size(accel);
for idx = 1:m
  phi(idx) = phi_of_accel(accel(:,idx));
  theta(idx) = theta_of_accel(accel(:,idx));
end;

m_phi = mean(phi)
m_theta = mean(theta)

std_dev_phi = std(phi)
std_dev_theta = std(theta)

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

m_psi = mean(psi)
std_dev_psi = std(psi)

subplot(3, 2, 5);
plot(psi);
title('psi (mag)');
subplot(3, 2, 6);
hist(psi);
title(sprintf('mean %f stddev %f', m_psi, std_dev_psi));
