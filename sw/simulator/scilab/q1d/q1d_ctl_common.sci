REF_Z   = 1;
REF_ZD  = 2;
REF_ZDD = 3;
REF_SIZE = 3;

//
// Parameters
//

ctl_max_err_z = 5;  // m
ctl_kp = -50/20;
ctl_kd = -10/20;

ctl_max_cmd = 0.99;  // actuators high saturation
ctl_min_cmd = 0.01;  // actuators low saturation

ref_omega = rad_of_deg(200.);// second order linear dynamics
ref_zeta =  0.85;
ref_saturate = 1;            // with saturations
ref_max_accel =  2.  * 9.81; // m/s2  - aka (fdm_max_thrust/fdm_mass - 1) fdm_g
ref_min_accel = -0.9 * 9.81; // m/s2
ref_max_speed =  3.;         // m/s
ref_min_speed = -3.;         // m/s
ref_thau_z  = 0.35;
ref_thau_zd = 0.12;

//
//  propagation of a second order linear model with saturations
//
function [Xrefi1] = ctl_update_ref(Xrefi, Xspi, dt)

  Xrefi1 = zeros(FDM_SIZE, 1);

  Xrefi1(REF_Z)   = Xrefi(REF_Z)  + dt * Xrefi(REF_ZD);
  Xrefi1(REF_ZD)  = Xrefi(REF_ZD) + dt * Xrefi(REF_ZDD);

  err_z = Xrefi1(REF_Z) - Xspi;
  sp_zd = -1/ref_thau_z*err_z;
  if sp_zd >= 0
    sp_zd = min(sp_zd, ref_max_speed);
  else
    sp_zd = max(sp_zd, ref_min_speed);
  end
  err_zd = Xrefi1(REF_ZD) - sp_zd;
  sp_zdd = -1/ref_thau_zd*err_zd;
  if sp_zdd >= 0
    sp_zdd = min(sp_zdd, ref_max_accel);
  else
    sp_zdd = max(sp_zdd, ref_min_accel);
  end
  Xrefi1(REF_ZDD) = sp_zdd;

endfunction

//
//
// Display
//
//

function ctl_display_ref(ctl_sp, ctl_ref_state, time)
nr = 3;
nc = 1;

subplot(nr,nc,1);
//plot2d(time, ctl_sp, 5);
//plot2d(time, ctl_ref_state(REF_Z,:),2);
_x = [time; time]';
_y = [ ctl_sp; ctl_ref_state(REF_Z,:)]';
plot2d(_x, _y, leg="input@ref", style=[3,2]);
xtitle('Z');//, 'time in s', 'm');

subplot(nr,nc,2);
plot2d(time, ctl_ref_state(REF_ZD,:),2);
plot2d(time,  ref_max_speed * ones(1, length(time)), 5);
plot2d(time, -ref_max_speed * ones(1, length(time)), 5);
xtitle('ZD');

subplot(nr,nc,3);
plot2d(time, ctl_ref_state(REF_ZDD,:),2);
plot2d(time,  ref_max_accel * ones(1, length(time)), 5);
plot2d(time,  ref_min_accel * ones(1, length(time)), 5);
xtitle('ZDD');


endfunction


function  ctl_display(ctl_type, ctl_list, ctl_sp, ctl_ref_state, ins_state, ctl_adp_state, ctl_adp_cov, ctl_adp_state, time)

  nr = 3;
  nc = 2;

  subplot(nr,nc,1);
  _rect = [0, -0.2, 5, .7];
  plot2d(time, ins_state(INS_Z,:),3);
  plot2d(time, ctl_ref_state(REF_Z,:),2, rect=_rect);
  plot2d(time, ctl_sp, 5);
  legends(["setpoint", "reference", "achieved"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Altitude');

  subplot(nr,nc,3);
  plot2d(time, ins_state(INS_ZD,:),3);
  plot2d(time, ctl_ref_state(REF_ZD,:),2);
  legends(["reference", "achieved"],[2 3], with_box=%f, opt="ur");
  xtitle('Vertical Speed');

  subplot(nr,nc,5);
  plot2d(time, ins_state(INS_ZDD,:),3);
  plot2d(time, ctl_ref_state(REF_ZDD,:),2);
  legends(["reference", "achieved"],[2 3], with_box=%f, opt="ur");
  xtitle('Vertical Acceleration');

  select ctl_type
  case CTL_MIAC
    subplot(nr,nc,2);
//    _rect = [time(1), 20, time($), 500];
//    plot2d(time, ctl_adp_meas,3, rect=_rect);
//    plot2d(time, ctl_adp_state,2, rect=_rect);
    plot2d(time, ctl_adp_meas,3);
    plot2d(time, ctl_adp_state,2);
    legends(["estimation", "measure"],[2 3], with_box=%f, opt="ur");
    xtitle('Parameter adaptation');

    subplot(nr,nc,4);
    _rect = [time(1), 0., time($), 0.003];
    plot2d(time, ctl_adp_cov,2, rect=_rect);
//    plot2d(time, ctl_adp_cov,2);
    xtitle('Parameter adaptation covariance');
  case CTL_MRAC
    subplot(nr,nc,2);
    _rect = [time(1), 1, time($), 4];
    plot2d(time, ctl_adp_state,2, rect=_rect);
    legends(["estimation"],[2 ], with_box=%f, opt="ur");
    xtitle('Parameter adaptation');
  end

  subplot(nr,nc,6);
  xset("color",5);
  xfpoly(ctl_list(5),ctl_list(3));
  xset("color",8);
  xfpoly(ctl_list(5),ctl_list(4));
  xset("color",1);
  //plot2d(time, ctl_list(1), 1);

  _rect = [time(1), 0, time($), 1.0];
  plot2d(time, ctl_list(2), 2,  rect=_rect);
//  plot2d(time, ctl_list(2), 2);

  legends(["feedforward", "feedback"],[2 5], with_box=%f, opt="ur");
  xtitle('Controler Output');

endfunction

