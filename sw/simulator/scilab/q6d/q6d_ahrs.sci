

AHRS_QI = 1;
AHRS_QX = 2;
AHRS_QY = 3;
AHRS_QZ = 4;
AHRS_BP = 5;
AHRS_BQ = 6;
AHRS_BR = 7;
AHRS_SSIZE = 7;

global ahrs_state;
global ahrs_euler;
global ahrs_rate;

function ahrs_init()

  global fdm_time;

  global ahrs_state;
  ahrs_state = zeros(AHRS_SSIZE, length(fdm_time));
  ahrs_state(AHRS_QI, 1) = 1;
  global ahr_euler;
  ahrs_euler = zeros(AXIS_NB, length(fdm_time));
  global ahrs_rate;
  ahrs_rate = zeros(AXIS_NB, length(fdm_time));

endfunction



function ahrs_propagate(i)

  global sensor_gyro;
  global ahrs_state;
  global ahrs_euler;
  global ahrs_rate;

  ahrs_state(AHRS_BP:AHRS_BR, i) = ahrs_state(AHRS_BP:AHRS_BR, i-1);
  ahrs_rate(:,i) = sensor_gyro(:,i) - ahrs_state(AHRS_BP:AHRS_BR, i-1);

  W = get_omega_quat(ahrs_rate(:,i-1));
  K_lagrange = 1.;
  quat = fdm_state(FDM_SQI:FDM_SQZ);
  quat_dot =  -1/2 * W * quat;
  quat_dot = quat_dot + K_lagrange * ( 1 - norm(quat)) * quat;
  dt = 1/512;
  ahrs_state(AHRS_QI:AHRS_QZ, i) = ahrs_state(AHRS_QI:AHRS_QZ, i-1) + dt * quat_dot;

  ahrs_euler(:,i) = euler_of_quat(ahrs_state(AHRS_QI:AHRS_QZ,i));

endfunction

function ahrs_display()
  nr = 3;
  nc = 3;
  global fdm_state;
  global fdm_euler;
  global fdm_raccel;

  global fdm_time;

  subplot(nr,nc,1);
  plot2d(fdm_time, deg_of_rad(fdm_euler(EULER_PHI,:)),3);
  plot2d(fdm_time, deg_of_rad(ahrs_euler(EULER_PHI,:)),2);
  legends(["setpoint", "ahrs", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Phi');
  subplot(nr,nc,2);
  plot2d(fdm_time, deg_of_rad(fdm_euler(EULER_THETA,:)),3);
  plot2d(fdm_time, deg_of_rad(ahrs_euler(EULER_THETA,:)),2);
  legends(["setpoint", "ahrs", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Theta');
  subplot(nr,nc,3);
  plot2d(fdm_time, deg_of_rad(fdm_euler(EULER_PSI,:)),3);
  plot2d(fdm_time, deg_of_rad(ahrs_euler(EULER_PSI,:)),2);
  legends(["setpoint", "ahrs", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Psi');

  subplot(nr,nc,4);
  plot2d(fdm_time, deg_of_rad(fdm_state(FDM_SP,:)),3);
legends(["setpoint", "ahrs", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('P');
  subplot(nr,nc,5);
  plot2d(fdm_time, deg_of_rad(fdm_state(FDM_SQ,:)),3);
legends(["setpoint", "ahrs", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Q');
  subplot(nr,nc,6);
  plot2d(fdm_time, deg_of_rad(fdm_state(FDM_SR,:)),3);
legends(["setpoint", "ahrs", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('R');

  subplot(nr,nc,7);
  plot2d(fdm_time, fdm_raccel(AXIS_X,:),3);
legends(["setpoint", "reference", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Pd');
  subplot(nr,nc,8);
  plot2d(fdm_time, fdm_raccel(AXIS_Y,:),3);
legends(["setpoint", "reference", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Qd');
  subplot(nr,nc,9);
  plot2d(fdm_time, fdm_raccel(AXIS_Z,:),3);
legends(["setpoint", "reference", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Rd');



endfunction
