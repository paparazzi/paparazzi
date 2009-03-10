


global ctl_sp_pos;
global ctl_ref_0;
global ctl_ref_1;
global ctl_ref_2;
global ctl_ref_3;
global ctl_ref_4;






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
  
endfunction





function ctl_run(i)

 global ctl_sp_pos;
 if fdm_time(i) < 10
   ctl_sp_pos(:,i)= [ 5; 0];
 else
   ctl_sp_pos(:,i)= [ 0; 1];
 end
 
 ctl_update_ref(i);
  
  
endfunction


ctl_omega_ref1 = [ rad_of_deg(90); rad_of_deg(90)]; 
ctl_zeta_ref1  = [ 0.85; 0.85 ]; 

ctl_omega_ref2 = [ rad_of_deg(1440); rad_of_deg(1440)]; 
ctl_zeta_ref2  = [ 0.85; 0.85 ]; 

a0 = ctl_omega_ref1^2 .* ctl_omega_ref2^2;
a1 = 2 * ctl_zeta_ref1 .* ctl_omega_ref1 .* ctl_omega_ref2^2 + ...
     2 * ctl_zeta_ref2 .* ctl_omega_ref2 .* ctl_omega_ref1^2; 
a2 = ctl_omega_ref1^2 + ...
     2 * ctl_zeta_ref1 .* ctl_omega_ref1 .* ctl_zeta_ref2 .* ctl_omega_ref2 + ...
     ctl_omega_ref2^2;
a3 = 2 * ctl_zeta_ref1 .* ctl_omega_ref1 + 2 * ctl_zeta_ref2 .* ctl_omega_ref2;
a4 = [1;1];




function ctl_update_ref(i)
  global ctl_sp_pos;
  global ctl_ref_0;
  global ctl_ref_1;
  global ctl_ref_2;
  global ctl_ref_3;
  global ctl_ref_4;

  err_pos = ctl_ref_0(:,i-1) - ctl_sp_pos(:,i-1);
  ctl_ref_4(:,i) = -a3 .* ctl_ref_3(:,i-1) -a2 .* ctl_ref_2(:,i-1) -a1 .* ctl_ref_1(:,i-1) -a0.*err_pos;
  
  dt = 1/512;
  ctl_ref_3(:,i) = ctl_ref_3(:,i-1) + dt * ctl_ref_4(:,i-1);
  ctl_ref_2(:,i) = ctl_ref_2(:,i-1) + dt * ctl_ref_3(:,i-1);
  ctl_ref_1(:,i) = ctl_ref_1(:,i-1) + dt * ctl_ref_2(:,i-1);
  ctl_ref_0(:,i) = ctl_ref_0(:,i-1) + dt * ctl_ref_1(:,i-1);

endfunction


function ctl_display()
  nr = 5;
  nc = 3;
  subplot(nr,nc,1);
  plot2d(fdm_time, fdm_state(FDM_SX,:),2);
  plot2d(fdm_time, ctl_ref_0(AXIS_X,:),3);
  plot2d(fdm_time, ctl_sp_pos(AXIS_X,:),5);
  legends(["setpoint", "fdm", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('X(0)');

  subplot(nr,nc,2);
  plot2d(fdm_time, fdm_state(FDM_SZ,:),2);
  plot2d(fdm_time, ctl_ref_0(AXIS_Z,:),3);
  plot2d(fdm_time, ctl_sp_pos(AXIS_Z,:),5);
  legends(["setpoint", "fdm", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Z(0)');

  subplot(nr,nc,3);
  plot2d(fdm_time, fdm_state(FDM_STHETA,:),2);
//  plot2d(fdm_time, ctl_ref_0(AXIS_Z,:),3);
//  plot2d(fdm_time, ctl_sp_pos(AXIS_Z,:),5);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Theta(0)');

  subplot(nr,nc,4);
//  plot2d(fdm_time, ins_state(INS_SX,:),2);
  plot2d(fdm_time, ctl_ref_1(AXIS_X,:),3);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('X(1)');

  subplot(nr,nc,5);
//  plot2d(fdm_time, ins_state(INS_SX,:),2);
  plot2d(fdm_time, ctl_ref_1(AXIS_Z,:),3);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Z(1)');

  subplot(nr,nc,6);
  plot2d(fdm_time, fdm_state(FDM_STHETAD,:),2);
//  plot2d(fdm_time, ctl_ref_0(AXIS_Z,:),3);
//  plot2d(fdm_time, ctl_sp_pos(AXIS_Z,:),5);
  legends(["setpoint", "INS", "ref"],[5 2 3], with_box=%f, opt="ur");
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

endfunction

