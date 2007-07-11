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
  gyro_supply =         [5.00   ;  5.00   ; 5.00    ]; // volt
  gyro_neutral_offset = [0.01   ; -0.01   ; 0.05    ]; // volt
  gyro_gains =          [0.0049 ;  0.0051 ; 0.004876]; // volt/deg/s 
  gyro_sigma_noise =    [0.003  ;  0.003  ; 0.003   ]; // volt^2

  gyro_adc_resolution = 2^10 * [1; 1; 1]; 
  
  gyro_rot_deg = [0   0   0
                  0   0  88   
		 -3  93   0 ];

  accel_supply =         [3.30   ;  3.30   ; 3.30    ]; // volt
  accel_neutral_offset = [0.01   ; -0.01   ; 0.05    ]; // volt
  accel_gains =          [0.3    ;  0.301  ; 0.299   ]; // volt/g 
  accel_sigma_noise =    [0.008  ;  0.008  ; 0.008   ]; // volt^2

  accel_adc_resolution = 2^10 * [1; 1; 1]; 
  
  
  
  DCM_GX = dcm_of_euler(rad_of_deg(gyro_rot_deg(:,1)));
  DCM_GY = dcm_of_euler(rad_of_deg(gyro_rot_deg(:,2)));
  DCM_GZ = dcm_of_euler(rad_of_deg(gyro_rot_deg(:,3)));

  gyro_align = [ DCM_GX(1,:)
                 DCM_GY(1,:)
	         DCM_GZ(1,:) ];
	     
  accel = [];
  mag = [];
  gyro = [];
  for i=1:length(time),
    g_volt = 0.5 * gyro_supply + gyro_neutral_offset + ...
	     (gyro_align * deg_of_rad(rates(:, i))) .* gyro_gains + ...
	     rand(3, 1, "normal") .* gyro_sigma_noise ;
    g_adc = round(g_volt ./ gyro_supply .* gyro_adc_resolution); 
    gyro = [gyro g_adc];
    
    DCM = dcm_of_euler(eulers(:,i));
    
    a_volt = 0.5 * accel_supply + accel_neutral_offset + ...
	         (DCM * [0; 0; 1]) .* accel_gains  + ...
	     rand(3, 1, "normal") .* accel_sigma_noise ;
    a_adc = round(a_volt ./ accel_supply .* accel_adc_resolution); 
    accel = [accel a_adc];	    
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

