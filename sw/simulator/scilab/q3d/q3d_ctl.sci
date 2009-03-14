


global ctl_sp_pos;
global ctl_ref_0;
global ctl_ref_1;
global ctl_ref_2;
global ctl_ref_3;
global ctl_ref_4;

global ctl_ref_theta;
global ctl_ref_thetad;

CMD_SF = 1;
CMD_DF = 2;

global ctl_cmd;
global ctl_motor;




function ctl_init()

  global fdm_time;
  
  global ctl_sp_pos;
  ctl_sp_pos = zeros(AXIS_NB, length(fdm_time));
  global ctl_ref_0;
  ctl_ref_0 = zeros(AXIS_NB, length(fdm_time));
  global ctl_ref_1;
  ctl_ref_1 = zeros(AXIS_NB, length(fdm_time));
  global ctl_ref_2;
  ctl_ref_2 = zeros(AXIS_NB, length(fdm_time));
  global ctl_ref_3;
  ctl_ref_3 = zeros(AXIS_NB, length(fdm_time));
  global ctl_ref_4;
  ctl_ref_4 = zeros(AXIS_NB, length(fdm_time));
  
  global ctl_ref_theta;
  ctl_ref_theta = zeros(1, length(fdm_time));
  global ctl_ref_thetad;
  ctl_ref_thetad = zeros(1, length(fdm_time));
  
  
  global ctl_cmd;
  ctl_cmd =  zeros(FDM_MOTOR_NB, length(fdm_time));
  global ctl_motor;
  ctl_motor = zeros(FDM_MOTOR_NB, length(fdm_time));
  
endfunction



ctl_mass    = 0.25;
ctl_inertia = 0.0078;

function ctl_run(i, sp)

 global ctl_sp_pos;
 ctl_sp_pos(:,i) = sp;
 
 ctl_update_ref_4th_order(i);

 global ctl_cmd;
 global ctl_ref_2;
 ctl_cmd(CMD_SF,i) = ctl_mass * sqrt(ctl_ref_2(AXIS_X,i)^2 +...
                                     (ctl_ref_2(AXIS_Z,i)+9.81)^2);

 x2   = ctl_ref_2(AXIS_X,i);				 
 z2p1 = ctl_ref_2(AXIS_Z,i)+9.81;				 
 x3   = ctl_ref_3(AXIS_X,i);				 
 z3   = ctl_ref_3(AXIS_Z,i);				 
 x4   = ctl_ref_4(AXIS_X,i);				 
 z4   = ctl_ref_4(AXIS_Z,i);				 
 a = x4*z2p1 - z4*x2; 				 
 b = z2p1^2+x2^2;
 c = 2 * (z2p1*z3 + x2*x3);
 d = x3*z2p1-z3*x2;
 ctl_cmd(CMD_DF,i) = -ctl_inertia * ( a/b - c*d/b^2);
 
 global ctl_motor;
 A2M = 0.5 * [ 1  1
               1 -1 ];
 ctl_motor(:,i) = A2M * ctl_cmd(:,i);
	
endfunction




ctl_omega_ref1 = [ rad_of_deg(90); rad_of_deg(90)]; 
ctl_zeta_ref1  = [ 0.9; 0.9 ]; 

ctl_omega_ref2 = [ rad_of_deg(1440); rad_of_deg(1440)]; 
ctl_zeta_ref2  = [ 0.9; 0.9 ]; 

a0 = ctl_omega_ref1^2 .* ctl_omega_ref2^2;
a1 = 2 * ctl_zeta_ref1 .* ctl_omega_ref1 .* ctl_omega_ref2^2 + ...
     2 * ctl_zeta_ref2 .* ctl_omega_ref2 .* ctl_omega_ref1^2; 
a2 = ctl_omega_ref1^2 + ...
     2 * ctl_zeta_ref1 .* ctl_omega_ref1 .* ctl_zeta_ref2 .* ctl_omega_ref2 + ...
     ctl_omega_ref2^2;
a3 = 2 * ctl_zeta_ref1 .* ctl_omega_ref1 + 2 * ctl_zeta_ref2 .* ctl_omega_ref2;
//a4 = [1;1];

function ctl_update_ref_4th_order(i)
  global ctl_sp_pos;
  global ctl_ref_0;
  global ctl_ref_1;
  global ctl_ref_2;
  global ctl_ref_3;
  global ctl_ref_4;

  dt = 1/512;
  ctl_ref_3(:,i) = ctl_ref_3(:,i-1) + dt * ctl_ref_4(:,i-1);
  ctl_ref_2(:,i) = ctl_ref_2(:,i-1) + dt * ctl_ref_3(:,i-1);
  ctl_ref_1(:,i) = ctl_ref_1(:,i-1) + dt * ctl_ref_2(:,i-1);
  ctl_ref_0(:,i) = ctl_ref_0(:,i-1) + dt * ctl_ref_1(:,i-1);

  err_pos = ctl_ref_0(:,i) - ctl_sp_pos(:,i);
  ctl_ref_4(:,i) = -a3 .* ctl_ref_3(:,i) -a2 .* ctl_ref_2(:,i) -a1 .* ctl_ref_1(:,i) -a0.*err_pos;
  
  global ctl_ref_theta;
  ctl_ref_theta(i) = -atan(ctl_ref_2(AXIS_X,i), 9.81 + ctl_ref_2(AXIS_Z,i));
  global ctl_ref_thetad;
  ctl_ref_thetad(i) = -((9.81 + ctl_ref_2(AXIS_Z,i))*ctl_ref_3(AXIS_X,i) - ctl_ref_2(AXIS_X,i)*ctl_ref_3(AXIS_Z,i)) / ...
                      ((9.81 + ctl_ref_2(AXIS_Z,i))^2+ctl_ref_2(AXIS_X,i)^2);
  
  
  
endfunction


function ctl_update_ref_1st_order()



endfunction



function ctl_display()
  nr = 5;
  nc = 3;
  subplot(nr,nc,1);
  plot_with_min_rect(fdm_time, fdm_state(FDM_SX,:),2, -0.5, 0.5);
//  plot2d(fdm_time, fdm_state(FDM_SX,:),2);
  plot2d(fdm_time, ctl_ref_0(AXIS_X,:),3);
  plot2d(fdm_time, ctl_sp_pos(AXIS_X,:),5);
  legends(["setpoint", "fdm", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('X(0)');

  subplot(nr,nc,2);
  plot_with_min_rect(fdm_time, fdm_state(FDM_SZ,:),2, -0.5, 0.5);
//  plot2d(fdm_time, fdm_state(FDM_SZ,:),2);
  plot2d(fdm_time, ctl_ref_0(AXIS_Z,:),3);
  plot2d(fdm_time, ctl_sp_pos(AXIS_Z,:),5);
  legends(["setpoint", "fdm", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Z(0)');

  subplot(nr,nc,3);
  plot_with_min_rect(fdm_time, deg_of_rad(fdm_state(FDM_STHETA,:)),2, -1., 1.);
//  plot2d(fdm_time, deg_of_rad(fdm_state(FDM_STHETA,:)),2);
  plot2d(fdm_time, deg_of_rad(ctl_ref_theta),3);
//  plot2d(fdm_time, ctl_sp_pos(AXIS_Z,:),5);
  legends(["setpoint", "fdm", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Theta(0)');

  subplot(nr,nc,4);
  plot_with_min_rect(fdm_time, fdm_state(FDM_SXD,:),2, -0.5, 0.5);
//  plot2d(fdm_time, fdm_state(FDM_SXD,:),2);
  plot2d(fdm_time, ctl_ref_1(AXIS_X,:),3);
  legends(["setpoint", "fdm", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('X(1)');

  subplot(nr,nc,5);
  plot_with_min_rect(fdm_time, fdm_state(FDM_SZD,:),2, -0.5, 0.5);
//  plot2d(fdm_time, fdm_state(FDM_SZD,:),2);
  plot2d(fdm_time, ctl_ref_1(AXIS_Z,:),3);
  legends(["setpoint", "fdm", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Z(1)');

  subplot(nr,nc,6);
   plot_with_min_rect(fdm_time, deg_of_rad(fdm_state(FDM_STHETAD,:)),2, -1., 1.);
   //plot2d(fdm_time, deg_of_rad(fdm_state(FDM_STHETAD,:)),2);
  plot2d(fdm_time, deg_of_rad(ctl_ref_thetad),3);
//  plot2d(fdm_time, ctl_sp_pos(AXIS_Z,:),5);
  legends(["setpoint", "fdm", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Theta(1)');

  subplot(nr,nc,7);
//  plot2d(fdm_time, ins_state(INS_SX,:),2);
  plot2d(fdm_time, ctl_ref_2(AXIS_X,:),3);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('X(2)');

  subplot(nr,nc,8);
//  plot2d(fdm_time, ins_state(INS_SX,:),2);
  plot2d(fdm_time, ctl_ref_2(AXIS_Z,:),3);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Z(2)');

  subplot(nr,nc,10);
//  plot2d(fdm_time, ins_state(INS_SX,:),2);
  plot2d(fdm_time, ctl_ref_3(AXIS_X,:),3);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('X(3)');

  subplot(nr,nc,11);
//  plot2d(fdm_time, ins_state(INS_SX,:),2);
  plot2d(fdm_time, ctl_ref_3(AXIS_Z,:),3);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Z(3)');

  subplot(nr,nc,13);
//  plot2d(fdm_time, ins_state(INS_SX,:),2);
  plot2d(fdm_time, ctl_ref_4(AXIS_X,:),3);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('X(4)');

  subplot(nr,nc,14);
//  plot2d(fdm_time, ins_state(INS_SX,:),2);
  plot2d(fdm_time, ctl_ref_4(AXIS_Z,:),3);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Z(4)');

  subplot(nr,nc,9);
  plot2d(fdm_time, ctl_cmd(CMD_SF,:),3);
//  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('U_sigma');
  
  subplot(nr,nc,12);
  plot2d(fdm_time, ctl_cmd(CMD_DF,:),3);
//  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('U_delta');
  
  subplot(nr,nc,15);
  plot2d(fdm_time, ctl_motor(FDM_MOTOR_RIGHT,:),2);
  plot2d(fdm_time, ctl_motor(FDM_MOTOR_LEFT,:),3);
  plot2d(fdm_time, fdm_min_thrust * ones(1,length(fdm_time)),5);
  plot2d(fdm_time, fdm_max_thrust * ones(1,length(fdm_time)),5);
//  legends(["setpoint", "RIGHT", "LEFT"],[5 2 3], with_box=%f, opt="ur");
  xtitle('U_motors');
  
endfunction

