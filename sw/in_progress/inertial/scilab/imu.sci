//
//
//
// Log parsing
//
//
//
function [time, accel, mag, gyro] = imu_read_log(filename)

time=[];
accel=[];
mag=[];
gyro=[];
dt = 0.015625;

u=mopen(filename, 'r');
idx = 0;
while meof(u) == 0,
  line = mgetl(u, 1);

  if strindex(line, 'IMU_GYRO') == 1
    [nb_scan, gp, gq, gr] = msscanf(1, line, 'IMU_GYRO %f %f %f');
    gyro = [gyro [gp;gq;gr]];
  end
  
  if strindex(line, 'IMU_ACCEL') == 1
    [nb_scan, ax, ay, az] = msscanf(1, line, 'IMU_ACCEL %f %f %f');
    accel = [accel [ax;ay;az]];
  end
  
  if strindex(line, 'IMU_MAG') == 1
    [nb_scan, mx, my, mz] = msscanf(1, line, 'IMU_MAG %f %f %f');
    mag = [mag [mx;my;mz]];
    time = [time idx*dt];
    idx = idx + 1;
  end

end
mclose(u);

endfunction



//
//
//
// Sensors sim
//
//
//
function [accel, mag, gyro] = imu_sim(time, rates, eulers)
  accel = [];
  mag = [];
  gyro = [];
  sigma_gyro = [0.01; 0.01; 0.01];
  bias_gyro = [0.05; 0.03; 0.04];

  g0 = [0.; 0.; 9.81];
  sigma_accel = [0.05; 0.05; 0.05];
  
  h0 = [185.; 0.; 157.];
  sigma_mag = [3.; 3.; 3.];
  
  for i=1:length(time),
    noise_gyro = rand(3, 1, "normal") .* sigma_gyro;
    g = rates(:, i) + noise_gyro + bias_gyro;
    gyro = [gyro g];
    
    DCM = dcm_of_euler(eulers(:,i));
    noise_accel = rand(3, 1, "normal") .* sigma_accel;
    acc = DCM * g0 + noise_accel;
    accel = [accel acc];
    
    noise_mag = rand(3, 1, "normal") .* sigma_mag;
    ma = DCM * h0 + noise_mag;
    mag = [mag ma];
  end
endfunction
