



global stabilization_sp_thrust;
global stabilization_sp_quat;
global stabilization_sp_euler;
global stabilization_ref_quat;
global stabilization_ref_euler;
global stabilization_ref_rate;
global stabilization_ref_raccel;

global stabilization_cmd_ff;
global stabilization_cmd_fb;
global stabilization_cmd_axis;
global stabilization_cmd_motors;


stab_omega_ref = [ rad_of_deg(720); rad_of_deg(720); rad_of_deg(720) ];
stab_zeta_ref  = [ 0.8; 0.8; 0.8 ];


function stabilization_init()

  global fdm_time;

  global stabilization_sp_thrust;
  global stabilization_sp_quat;
  global stabilization_sp_euler;
  global stabilization_ref_quat;
  global stabilization_ref_euler;
  global stabilization_ref_rate;
  global stabilization_ref_raccel;

  stabilization_sp_thrust = zeros(1, length(fdm_time));
  stabilization_sp_quat = zeros(Q_SIZE, length(fdm_time));
  stabilization_sp_euler = zeros(EULER_NB, length(fdm_time));
  stabilization_ref_quat = zeros(Q_SIZE, length(fdm_time));
  stabilization_ref_quat(Q_QI, 1) = 1.;
  stabilization_ref_euler = zeros(EULER_NB, length(fdm_time));
  stabilization_ref_rate = zeros(AXIS_NB, length(fdm_time));
  stabilization_ref_raccel = zeros(AXIS_NB, length(fdm_time));

  global stabilization_cmd_ff;
  global stabilization_cmd_fb;
  global stabilization_cmd_axis;
  global stabilization_cmd_motors;
  stabilization_cmd_ff = zeros(AXIS_NB, length(fdm_time));
  stabilization_cmd_fb = zeros(AXIS_NB, length(fdm_time));
  stabilization_cmd_axis = zeros(4, length(fdm_time));
  stabilization_cmd_motors = zeros(4, length(fdm_time));

endfunction

Kp  = [ -0.5; -0.5; -0.5 ];
Kd  = [ -0.5; -0.5; -0.5 ];
Kdd = [  0.007;  0.007;  0.007 ];

function stabilization_run(i)

  global ahrs_state;
  global stabilization_sp_euler;
  global stabilization_sp_quat;
  stabilization_sp_euler(:,i) = euler_of_quat(stabilization_sp_quat(:,i));

  stabilization_update_ref(i);

  err_angle = quat_div(stabilization_ref_quat(:,i), ahrs_state(AHRS_QI:AHRS_QZ,i));
  err_rate = ahrs_rate(:,i) - stabilization_ref_rate(:,i);

  global stabilization_cmd_fb;
  stabilization_cmd_fb(:,i) = Kp .* err_angle(Q_QX:Q_QZ) + Kd .* err_rate ;
  global stabilization_cmd_ff;
  stabilization_cmd_ff(:,i) = Kdd .* stabilization_ref_raccel(:,i);
  global stabilization_cmd_axis;
  global stabilization_sp_thrust;
  stabilization_cmd_axis(:,i) = [stabilization_cmd_ff(:,i)+stabilization_cmd_fb(:,i)
                                 stabilization_sp_thrust(i)];
  stabilization_mix(i);

endfunction


function stabilization_update_ref(i)

  global stabilization_sp_quat;
  global stabilization_ref_quat;
  global stabilization_ref_rate;
  global stabilization_ref_raccel;

//  pause

//  error_quat = quat_mult_inv(stabilization_sp_quat(:,i-1), stabilization_ref_quat(:,i-1));
//  error_quat = quat_inv_comp(stabilization_ref_quat(:,i-1), stabilization_sp_quat(:,i-1));
  error_quat = quat_div(stabilization_sp_quat(:,i-1), stabilization_ref_quat(:,i-1));
  error_quat = quat_wrap_shortest(error_quat);

  ref_accel_0 = -stab_omega_ref^2 .* error_quat(Q_QX:Q_QZ);
  ref_accel_1 = -2. * stab_zeta_ref .* stab_omega_ref .* stabilization_ref_rate(:,i-1);

  stabilization_ref_raccel(:,i) = ref_accel_0 + ref_accel_1;

  W = get_omega_quat(stabilization_ref_rate(:,i-1));
  K_lagrange = 1.;
  quat = stabilization_ref_quat(:,i-1);
  quat_dot =  -1/2 * W * quat;
  quat_dot = quat_dot + K_lagrange * ( 1 - norm(quat)) * quat;
  dt = 1/512;
  stabilization_ref_quat(:,i) = stabilization_ref_quat(:,i-1) + dt * quat_dot;

  stabilization_ref_rate(:,i) = stabilization_ref_rate(:,i-1) + dt * stabilization_ref_raccel(:,i-1);

  global stabilization_ref_euler;
  stabilization_ref_euler(:,i) = euler_of_quat(stabilization_ref_quat(:,i));

endfunction



function stabilization_mix(i)

  global stabilization_cmd_axis;
  global stabilization_cmd_motors;

  motors_of_axis = [  0  1 -1  1
                     -1  0  1  1
		      0 -1 -1  1
                      1  0  1  1 ];

  stabilization_cmd_motors(:,i) = motors_of_axis * stabilization_cmd_axis(:,i);

endfunction




function stabilization_display()

  nr = 3;
  nc = 3;
  global stabilization_ref_euler;
  global stabilization_sp_euler;

  global fdm_state;
  global fdm_euler;
  global fdm_time;

  subplot(nr,nc,1);
  plot2d(fdm_time, deg_of_rad(stabilization_ref_euler(EULER_PHI,:)),2);
  plot2d(fdm_time, deg_of_rad(stabilization_sp_euler(EULER_PHI,:)),5);
  plot2d(fdm_time, deg_of_rad(fdm_euler(EULER_PHI,:)),3);
  legends(["setpoint", "Ref", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('PHI');
  subplot(nr,nc,2);
  plot2d(fdm_time, deg_of_rad(stabilization_ref_euler(EULER_THETA,:)),2);
  plot2d(fdm_time, deg_of_rad(stabilization_sp_euler(EULER_THETA,:)),5);
  plot2d(fdm_time, deg_of_rad(fdm_euler(EULER_THETA,:)),3);
  legends(["setpoint", "Ref", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('THETA');
  subplot(nr,nc,3);
  plot2d(fdm_time, deg_of_rad(stabilization_ref_euler(EULER_PSI,:)),2);
  plot2d(fdm_time, deg_of_rad(stabilization_sp_euler(EULER_PSI,:)),5);
  plot2d(fdm_time, deg_of_rad(fdm_euler(EULER_PSI,:)),3);
  legends(["setpoint", "Ref", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('PSI');

  global stabilization_ref_rate;
  subplot(nr,nc,4);
  plot2d(fdm_time, deg_of_rad(stabilization_ref_rate(AXIS_X,:)),2);
  plot2d(fdm_time, deg_of_rad(fdm_state(FDM_SP,:)),3);
  legends(["setpoint", "Ref", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('P');
  subplot(nr,nc,5);
  plot2d(fdm_time, deg_of_rad(stabilization_ref_rate(AXIS_Y,:)),2);
  plot2d(fdm_time, deg_of_rad(fdm_state(FDM_SQ,:)),3);
  legends(["setpoint", "Ref", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Q');
  subplot(nr,nc,6);
  plot2d(fdm_time, deg_of_rad(stabilization_ref_rate(AXIS_Z,:)),2);
  plot2d(fdm_time, deg_of_rad(fdm_state(FDM_SR,:)),3);
  legends(["setpoint", "Ref", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('R');





  global stabilization_cmd_axis;
  global stabilization_cmd_motors;

  subplot(nr,nc,7);
  plot2d(fdm_time, stabilization_cmd_axis(AXIS_X,:),1);
  plot2d(fdm_time, stabilization_cmd_axis(AXIS_Y,:),2);
  plot2d(fdm_time, stabilization_cmd_axis(AXIS_Z,:),3);
  legends(["p", "q", "r"],[1 2 3], with_box=%f, opt="ur");
  xtitle('Cmd axis');

  subplot(nr,nc,8);
  plot2d(fdm_time, stabilization_cmd_motors(FDM_MOTOR_FRONT,:),1);
  plot2d(fdm_time, stabilization_cmd_motors(FDM_MOTOR_RIGHT,:),2);
  plot2d(fdm_time, stabilization_cmd_motors(FDM_MOTOR_BACK,:),3);
  plot2d(fdm_time, stabilization_cmd_motors(FDM_MOTOR_LEFT,:),4);
  legends(["f", "r", "b", "l"],[1 2 3 4], with_box=%f, opt="ur");
  xtitle('Cmd motor');

endfunction

