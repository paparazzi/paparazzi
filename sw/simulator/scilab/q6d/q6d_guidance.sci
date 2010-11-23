



global guidance_sp_psi;
global guidance_sp_pos;
global guidance_ref_pos;
global guidance_ref_speed;
global guidance_ref_accel;
global guidance_ref_acceld;


global guidance_cmd_ff;
global guidance_cmd_fb;
global guidance_output_quat;
global guidance_output_thrust;

guidance_omega_ref = [ rad_of_deg(90); rad_of_deg(90); rad_of_deg(270) ];
guidance_zeta_ref  = [ 0.8; 0.8; 0.8 ];

guidance_vsat = [ -3  3
                  -8 16
		  40 40 ];
guidance_hsat = [ -3  3
                  -3  3
		 -10 10 ];

guidance_thau = [ -1/1. -1/0.6 -1/0.25
                  -1/1. -1/0.6 -1/0.25
		  -1/0.36 -1/0.15 -1/0.7  ];

guidance_mass = 0.5;
guidance_Ct0  = 0.5*9.81*4;

guidance_Kp = 0.5* [ -0.1; -0.1; -0.1];
guidance_Kd = 1.5* [ -0.1; -0.1; -0.1];


function guidance_init()

  global fdm_time;

  global guidance_sp_psi;
  guidance_sp_psi = zeros(1, length(fdm_time));
  global guidance_sp_pos;
  guidance_sp_pos = zeros(AXIS_NB, length(fdm_time));
  global guidance_ref_pos;
  guidance_ref_pos = zeros(AXIS_NB, length(fdm_time));
  global guidance_ref_speed;
  guidance_ref_speed = zeros(AXIS_NB, length(fdm_time));
  global guidance_ref_accel;
  guidance_ref_accel = zeros(AXIS_NB, length(fdm_time));

  global guidance_output_quat;
  guidance_output_quat = zeros(Q_SIZE, length(fdm_time));
  global guidance_output_thrust;
  guidance_output_thrust = zeros(1, length(fdm_time));

endfunction

function guidance_run(i)

// guidance_step_phi(i);
// guidance_step_theta(i);
//  guidance_step_psi(i);
//  global guidance_output_thrust;
//  guidance_output_thrust(i) = 0.29;

//  guidance_step_x(i);
  guidance_step_y(i);
//  guidance_step_z(i);
  guidance_update_ref(i);
  guidance_hover(i);


  global stabilization_sp_thrust;
  global stabilization_sp_quat;
  global guidance_output_quat;
  global guidance_output_thrust;
  stabilization_sp_thrust(i) = guidance_output_thrust(i);
  stabilization_sp_quat(:,i) = guidance_output_quat(:, i);

endfunction




//
//
//

function guidance_hover(i)

  global guidance_ref_pos;
  global guidance_ref_speed;
  global guidance_ref_accel;
  global ins_state;
  global ahrs_state;

  pos_err = ins_state(INS_SX:INS_SZ,i) - guidance_ref_pos(:,i);
  speed_err = ins_state(INS_SXD:INS_SZD,i) - guidance_ref_speed(:,i);

  corr_thrust_ltp = guidance_Kp .* pos_err + guidance_Kd .* speed_err;

  nli_thrust_ltp = -guidance_mass / guidance_Ct0 * ( [ 0; 0; 9.81] - guidance_ref_accel(:,i));

  thrust_ltp =  nli_thrust_ltp + corr_thrust_ltp;

  axis_foo = cross_product(thrust_ltp/norm(thrust_ltp), [0; 0; -1]);

  angle_foo = asin(norm(axis_foo));

  quat_foo = [ cos(angle_foo); -axis_foo];

//  printf("%f %f %f\n", nli_thrust_ltp(1), nli_thrust_ltp(2), nli_thrust_ltp(3));
//  printf("%f %f %f %f\n", quat_foo(1), quat_foo(2), quat_foo(3), quat_foo(4));

//  thrust_body = quat_vect_mult(ahrs_state(AHRS_QI:AHRS_QZ, i), thrust_ltp);

  global guidance_output_thrust;
  guidance_output_thrust(i) = norm(thrust_ltp);

  global guidance_output_quat;
  guidance_output_quat(:, i) = quat_foo; //[1; 0; 0; 0];
endfunction


//
//
//
function guidance_update_ref_old(i)

  global guidance_sp_pos;
  global guidance_ref_pos;
  global guidance_ref_speed;
  global guidance_ref_accel;
  global guidance_ref_acceld;

  ref_err_pos = guidance_ref_pos(:,i-1) - guidance_sp_pos(:,i-1);

  guidance_ref_accel(:,i) = -guidance_omega_ref^2 .* ref_err_pos ...
                            -2*guidance_zeta_ref .* guidance_omega_ref .* guidance_ref_speed(:,i-1);
  dt = 1/512;
  guidance_ref_speed(:,i) = guidance_ref_speed(:,i-1) + dt * guidance_ref_accel(:,i-1);
  guidance_ref_pos(:,i) = guidance_ref_pos(:,i-1) + dt * guidance_ref_speed(:,i-1);

endfunction

function guidance_update_ref(i)

  global guidance_sp_pos;
  global guidance_ref_pos;
  global guidance_ref_speed;
  global guidance_ref_accel;
  global guidance_ref_acceld;

  dt = 1/512;
  guidance_ref_pos(:,i) = guidance_ref_pos(:,i-1) + dt * guidance_ref_speed(:,i-1);
  guidance_ref_speed(:,i) = guidance_ref_speed(:,i-1) + dt * guidance_ref_accel(:,i-1);
  guidance_ref_accel(:,i) = guidance_ref_accel(:,i-1) + dt * guidance_ref_acceld(:,i-1);


  err_pos = guidance_ref_pos(:,i-1) - guidance_sp_pos(:,i);
//  trim(err_pos, -5,  5);
  sp_speed = guidance_thau(:,1).*err_pos;
//  sp_speed(1) = trim(sp_speed(1), guidance_hsat(1,1),  guidance_hsat(1,2));
//  sp_speed(2) = trim(sp_speed(2), guidance_hsat(1,1),  guidance_hsat(1,2));
  sp_speed(3) = trim(sp_speed(3), guidance_vsat(1,1),  guidance_vsat(1,2));
  err_speed = guidance_ref_speed(:,i) - sp_speed;
  sp_accel = guidance_thau(:,2).*err_speed;
//  sp_accel(1) = trim(sp_accel(1), guidance_hsat(2,1),  guidance_hsat(2,2));
//  sp_accel(2) = trim(sp_accel(2), guidance_hsat(2,1),  guidance_hsat(2,2));
  sp_accel(3) = trim(sp_accel(3), guidance_vsat(2,1),  guidance_vsat(2,2));
  err_accel = guidance_ref_accel(:,i) - sp_accel;
  sp_acceld = guidance_thau(:,3).*err_accel;
//  sp_acceld(1) = trim(sp_acceld(1), guidance_hsat(3,1),  guidance_hsat(3,2));
//  sp_acceld(2) = trim(sp_acceld(2), guidance_hsat(3,1),  guidance_hsat(3,2));
  sp_acceld(3) = trim(sp_acceld(3), guidance_vsat(3,1),  guidance_vsat(3,2));
  guidance_ref_acceld(:,i) = sp_acceld;

endfunction


//
//
//
function guidance_step_x(i)
  global fdm_time;
  if modulo(i,6144) < 3072
    pos_sp = [ 10; 0; 0];
  else
    pos_sp = [ -10; 0; 0];
  end
  global guidance_sp_pos;
  guidance_sp_pos(:,i) = pos_sp;
endfunction

//
//
//
function guidance_step_y(i)
  global fdm_time;
  if modulo(i,6144) < 3072
    pos_sp = [ 0; 5; 0];
  else
    pos_sp = [ 0; -5; 0];
  end
  global guidance_sp_pos;
  guidance_sp_pos(:,i) = pos_sp;
endfunction

//
//
//
function guidance_step_z(i)
  global fdm_time;
  if modulo(i,1024) < 512
    pos_sp = [ 0; 0; -1];
  else
    pos_sp = [ 0; 0; 0];
  end
  global guidance_sp_pos;
  guidance_sp_pos(:,i) = pos_sp;
endfunction



//
//
//
function guidance_step_phi(i)
  global fdm_time;
  if modulo(i,512) < 256
    euler_sp = [ rad_of_deg(10); 0; 0];
  else
    euler_sp = [ rad_of_deg(-10); 0; 0];
  end
  global guidance_output_quat;
  guidance_output_quat(:, i) = quat_of_euler(euler_sp);
endfunction

//
//
//
function guidance_step_theta(i)
  global fdm_time;
  if modulo(i,512) < 256
   euler_sp = [ 0; rad_of_deg(10); 0];
  else
   euler_sp = [ 0; rad_of_deg(-10); 0];
  end
  global guidance_output_quat;
  guidance_output_quat(:, i) = quat_of_euler(euler_sp);
endfunction

//
//
//
function guidance_step_psi(i)
  global fdm_time;
  if modulo(i,512) < 256
    euler_sp = [ 0; 0; rad_of_deg(10)];
  else
    euler_sp = [ 0; 0; rad_of_deg(-10)];
  end
  global guidance_output_quat;
  guidance_output_quat(:, i) = quat_of_euler(euler_sp);
endfunction


//
//
//
function guidance_display()

  nr = 4;
  nc = 3;
  global ins_state;
  global ins_accel;
  global guidance_ref_pos;
  global guidance_ref_speed;
  global guidance_ref_accel;

  subplot(nr,nc,1);
  plot2d(fdm_time, ins_state(INS_SX,:),2);
  plot2d(fdm_time, guidance_ref_pos(AXIS_X,:),3);
  plot2d(fdm_time, guidance_sp_pos(AXIS_X,:),5);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('X');
  subplot(nr,nc,2);
  plot2d(fdm_time, ins_state(INS_SY,:),2);
  plot2d(fdm_time, guidance_ref_pos(AXIS_Y,:),3);
  plot2d(fdm_time, guidance_sp_pos(AXIS_Y,:),5);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Y');
  subplot(nr,nc,3);
  plot2d(fdm_time, ins_state(INS_SZ,:),2);
  plot2d(fdm_time, guidance_ref_pos(AXIS_Z,:),3);
  plot2d(fdm_time, guidance_sp_pos(AXIS_Z,:),5);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Z');

  subplot(nr,nc,4);
  plot2d(fdm_time, ins_state(INS_SXD,:),2);
  plot2d(fdm_time, guidance_ref_speed(AXIS_X,:),3);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Xdot');
  subplot(nr,nc,5);
  plot2d(fdm_time, ins_state(INS_SYD,:),2);
  plot2d(fdm_time, guidance_ref_speed(AXIS_Y,:),3);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Ydot');
  subplot(nr,nc,6);
  plot2d(fdm_time, ins_state(INS_SZD,:),2);
  plot2d(fdm_time, guidance_ref_speed(AXIS_Z,:),3);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Zdot');

  subplot(nr,nc,7);
  plot2d(fdm_time, ins_accel(AXIS_X,:),2);
  plot2d(fdm_time, guidance_ref_accel(AXIS_X,:),3);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Xdotdot');
  subplot(nr,nc,8);
  plot2d(fdm_time, ins_accel(AXIS_Y,:),2);
  plot2d(fdm_time, guidance_ref_accel(AXIS_Y,:),3);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Ydotdot');
  subplot(nr,nc,9);
  plot2d(fdm_time, ins_accel(AXIS_Z,:),2);
  plot2d(fdm_time, guidance_ref_accel(AXIS_Z,:),3);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Zdotdot');

  subplot(nr,nc,12);
  plot2d(fdm_time, guidance_output_thrust,3);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Thrust');

endfunction


