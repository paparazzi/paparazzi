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


function [accel, mag, gyro] = imu_sim_misaligned(time, rates, eulers)
  gyro_supply = 5.;
  gyro_neutral = [512; 512; 512];
  gyro_gains = rad_of_deg([ 300/512; 300/512; 300/512]);
  gyro_sigma_noise = [ 3; 3; 3]; 

  gyro_offset_deg = [0   0   0
                     0   0   0   
		     0   0   0 ];

  gyro_orientation = [] ;
		 
  gyro_align = [ 1   0 0
                 0.1 1 0
	         0   0 1 ];

  accel = [];
  mag = [];
  gyro = [];
  for i=1:length(time),
    gyro_noise = rand(3, 1, "normal") .* gyro_sigma_noise;
//    gyro_noise = [ 0; 0; 0];
    g = gyro_neutral + (gyro_align * rates(:, i)) ./ gyro_gains + gyro_noise;
    g = round(g);
    gyro = [gyro g];
  end
endfunction


//
//
//
// Angles Measurements
//
//
//
function [phi] = phi_of_accel(accel)
  phi = atan(accel(2), accel(3));
endfunction

function [theta] = theta_of_accel(accel)
  theta = -asin(accel(1) / norm(accel));   
endfunction

function [psi] = psi_of_mag(phi, theta, mag)
  cphi   = cos( phi );
  sphi   = sin( phi );
  ctheta = cos( theta );
  stheta = sin( theta );
  mn = ctheta*      mag(1)+ sphi*stheta* mag(2)+ cphi*stheta* mag(3);
  me = cphi*  mag(2) -sphi* mag(3);
  psi = -atan( me, mn );
endfunction

function [euler] = euler_of_accel_mag(accel, mag)

  phi = phi_of_accel(accel);
  theta = theta_of_accel(accel);
  psi = psi_of_mag(phi, theta, mag);

  euler = [ phi; theta; psi];

endfunction


function [m_eulers] = ahrs_compute_euler_measurements(accel, mag)
  m_eulers = [];
  [m n] = size(accel);
  for i=1:n
    phi = phi_of_accel(accel(:,i));
    theta = theta_of_accel(accel(:,i));
    psi = psi_of_mag(phi, theta, mag(:,i));
    m_eulers = [m_eulers [phi;theta;psi]];
  end

endfunction

