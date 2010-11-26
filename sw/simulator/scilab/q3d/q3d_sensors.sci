SEN_SAX   = 1;
SEN_SAZ   = 2;
SEN_SG    = 3;
SEN_SSIZE = 3;

global sensors_time;
global sensors_state;

gyro_noise = rad_of_deg(3.);
gyro_bias  = rad_of_deg(0.);
accel_noise = 0.5;

function sensors_init(time)

  global sensors_time;
  sensors_time = time;
  global sensors_state;
  sensors_state = zeros(SEN_SSIZE, length(time));

endfunction



function sensors_run(i)

  global sensors_state;

  accel_inertial = fdm_accel(FDM_AX:FDM_AZ, i) - [0, -9.81]';
  in_2_body = [ cos(fdm_state(FDM_STHETA, i))  sin(fdm_state(FDM_STHETA, i))
               -sin(fdm_state(FDM_STHETA, i))  cos(fdm_state(FDM_STHETA, i)) ];
  accel_body =  in_2_body * accel_inertial + accel_noise * rand(2,1,'normal');
  sensors_state(SEN_SAX:SEN_SAZ, i) = accel_body;
  sensors_state(SEN_SG, i) = fdm_state(FDM_STHETAD, i) + gyro_noise * rand(1,1,'normal') + gyro_bias;

endfunction

