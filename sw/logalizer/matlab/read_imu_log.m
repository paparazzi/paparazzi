function [gyro, accel, mag] = read_imu_log(filename)

gyro=[];
accel=[];
mag=[];

fid = fopen(filename, 'r');

while 1
  tline = fgetl(fid);
  if ~ischar(tline),   break,   end
  [A, count] = sscanf(tline, 'IMU_MAG %d %d %d');
  if (count == 3), mag = [mag A];, end;
  [A, count] = sscanf(tline, 'IMU_ACCEL %f %f %f');
  if (count == 3), accel = [accel A];, end;
  [A, count] = sscanf(tline, 'IMU_GYRO %f %f %f');
  if (count == 3), gyro = [gyro A];, end;
end

