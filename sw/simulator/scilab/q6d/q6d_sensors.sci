//
// Sensors model
//




global sensor_gyro;
global sensor_accel;
global sensor_baro;
global sensor_gps_pos;
global sensor_gps_speed;


function sensors_init() 

  global fdm_time;
  global sensor_accel;
  sensor_accel = zeros(AXIS_NB, length(fdm_time));
  global sensor_gyro;
  sensor_gyro = zeros(AXIS_NB, length(fdm_time)); 
  global sensor_baro;
  sensor_baro = zeros(1, length(fdm_time)); 
  global sensor_gps_pos;
  sensor_gps_pos = zeros(AXIS_NB, length(fdm_time)); 
  global sensor_gps_speed;
  sensor_gps_speed = zeros(AXIS_NB, length(fdm_time)); 
  
endfunction



function sensors_run(i)

  global fdm_state;
  global fdm_accel;
  global sensor_gyro;
  global sensor_accel;
  global sensor_baro;
  global sensor_gps_pos;
  global sensor_gps_speed;

  sensor_gyro(:,i) = fdm_state(FDM_SP:FDM_SR, i);

  sensor_accel(:,i) = quat_vect_mult(fdm_state(FDM_SQI:FDM_SQZ,i), (fdm_accel(:,i)-[0; 0; 9.81]));

  sensor_gps_pos(:,i) = fdm_state(FDM_SX:FDM_SZ, i);

  sensor_gps_speed(:,i) = fdm_state(FDM_SXD:FDM_SZD, i);
  
//  sensor_baro(:,i) = fdm_state(FDM_SZ, i);

endfunction

