
INS_SX    = 1;
INS_SY    = 2;
INS_SZ    = 3;
INS_SXD   = 4;
INS_SYD   = 5;
INS_SZD   = 6;
INS_SBX   = 7;
INS_SBY   = 8;
INS_SBZ   = 9;
INS_SSIZE = 9;


global ins_state;
global ins_accel;


function ins_init()

  global ins_state;
  ins_state = zeros(INS_SSIZE, length(fdm_time));
  global ins_accel;
  ins_accel = zeros(AXIS_NB, length(fdm_time));

endfunction

// propagate from i-1 to i
function ins_propagate(i)

  global ins_state;
  global ins_accel;
  global ahrs_state;
  global sensor_accel;

  // convert to LTP
  accel_ltp = quat_vect_inv_mult(ahrs_state(Q_QI:Q_QZ, i), sensor_accel(:,i-1));
  // correct for gravity
  accel_ltp = accel_ltp + [ 0; 0; 9.81];
  // store unbiased value
  ins_accel(:,i) = accel_ltp - ins_state(INS_SBX:INS_SBZ,i-1);
  // compute derivative
  state_dot = [ ins_state(INS_SXD:INS_SZD, i-1); ins_accel(:,i); 0; 0; 0];
  // propagate
  dt = fdm_time(i) - fdm_time(i-1);
  ins_state(:,i) = ins_state(:,i-1) + state_dot * dt;

endfunction


function ins_display()

  nr = 3;
  nc = 3;
  global ins_state;
  global fdm_state;
  global fdm_time;

  subplot(nr,nc,1);
  plot2d(fdm_time, fdm_state(FDM_SX,:),3);
  plot2d(fdm_time, ins_state(INS_SX,:),2);
  legends(["setpoint", "INS", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('X');
  subplot(nr,nc,2);
  plot2d(fdm_time, fdm_state(FDM_SY,:),3);
  plot2d(fdm_time, ins_state(INS_SY,:),2);
  legends(["setpoint", "INS", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Y');
  subplot(nr,nc,3);
  plot2d(fdm_time, fdm_state(FDM_SZ,:),3);
  plot2d(fdm_time, ins_state(INS_SZ,:),2);
  legends(["setpoint", "INS", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Z');

  subplot(nr,nc,4);
  plot2d(fdm_time, fdm_state(FDM_SXD,:),3);
  plot2d(fdm_time, ins_state(INS_SXD,:),2);
  legends(["setpoint", "INS", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Xdot');
  subplot(nr,nc,5);
  plot2d(fdm_time, fdm_state(FDM_SYD,:),3);
  plot2d(fdm_time, ins_state(INS_SYD,:),2);
  legends(["setpoint", "INS", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Ydot');
  subplot(nr,nc,6);
  plot2d(fdm_time, fdm_state(FDM_SZD,:),3);
  plot2d(fdm_time, ins_state(INS_SZD,:),2);
  legends(["setpoint", "INS", "fdm"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Zdot');

endfunction

