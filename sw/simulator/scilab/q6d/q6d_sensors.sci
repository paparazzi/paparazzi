//
// Sensors model
//

sensor_noise_gyro = rad_of_deg(3);
sensor_bias_gyro = [rad_of_deg(2.5) rad_of_deg(-2.5) rad_of_deg(-1.25)]';
//sensor_bias_gyro = [rad_of_deg(0.5) rad_of_deg(-0.5) rad_of_deg(0.25)]';
//sensor_bias_gyro = [rad_of_deg(0) rad_of_deg(0) rad_of_deg(0)]';
sensor_noise_accel = 1.;
sensor_noise_mag = 0.5;


global sensor_gyro;
global sensor_accel;
global sensor_mag;
global sensor_baro;
global sensor_gps_pos;
global sensor_gps_speed;


function sensors_init(time)

  global sensor_gyro;
  sensor_gyro = sensor_noise_gyro * rand(AXIS_NB, length(time),'normal');
  global sensor_accel;
  sensor_accel = sensor_noise_accel * rand(AXIS_NB, length(time),'normal');
  global sensor_mag;
  sensor_mag = sensor_noise_mag * rand(AXIS_NB, length(time),'normal');
  global sensor_baro;
  sensor_baro = zeros(1, length(time));
  global sensor_gps_pos;
  sensor_gps_pos = zeros(AXIS_NB, length(time));
  global sensor_gps_speed;
  sensor_gps_speed = zeros(AXIS_NB, length(time));

endfunction



function sensors_run(i)

  global fdm_state;
  global fdm_accel;
  global sensor_gyro;
  global sensor_accel;
  global sensor_mag;
  global sensor_baro;
  global sensor_gps_pos;
  global sensor_gps_speed;

  sensor_gyro(:,i) = sensor_gyro(:,i) + fdm_state(FDM_SP:FDM_SR, i) + sensor_bias_gyro;

  accel_earth = fdm_accel(:,i)-[0; 0; 9.81];
  accel_body = quat_vect_mult(fdm_state(FDM_SQI:FDM_SQZ,i), accel_earth);
  sensor_accel(:,i) = sensor_accel(:,i) + accel_body;

//  mag_earth = [1 0 1]';
  mag_earth = [0.4912 0.1225 0.8624]';
  mag_body = quat_vect_mult(fdm_state(FDM_SQI:FDM_SQZ,i), mag_earth);
  sensor_mag(:,i) = sensor_mag(:,i) + mag_body;

  sensor_gps_pos(:,i) = fdm_state(FDM_SX:FDM_SZ, i);

  sensor_gps_speed(:,i) = fdm_state(FDM_SXD:FDM_SZD, i);

//  sensor_baro(:,i) = fdm_state(FDM_SZ, i);

endfunction

