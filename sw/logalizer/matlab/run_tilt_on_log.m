clear
fid = fopen('data/log_ahrs_bug', 'r');

mag=[];
accel=[];
gyro=[];

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
end

%mag
%accel
%gyro
%ab_ahrs

%plot(mag(3,:))
%plot(gyro(3,:))
%plot(accel(3,:))

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
plot(saved_t, saved_theta, ...
     saved_t, theta_measure);
title('angle');

subplot(3,1,2)
plot (saved_t, gyro(1,:), saved_t, gyro(2,:), saved_t, gyro(3,:));
title('gyros');


subplot(3,1,3)
plot(saved_t, saved_by);
title('bias');


