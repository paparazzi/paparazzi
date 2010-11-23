//
// MRAC Adaptive control parameter
//

mrac_gma = 0.001;
mrac_lbd = (-ctl_kd + sqrt(ctl_kd^2-4*ctl_kp))/2;
mrac_c = (-ctl_kd - sqrt(ctl_kd^2-4*ctl_kp))/2;

function [Xadpi1p] = ctl_mrac(Xadpim1, Xinsi, Xrefim1, Uim1)

  // error control
  err_z  = Xinsi(INS_Z) - Xrefim1(REF_Z);
  err_z  = trim(err_z, -ctl_max_err_z, ctl_max_err_z);
  err_zd = Xinsi(INS_ZD) - Xrefim1(REF_ZD);
  // generalized error
  s = err_zd + mrac_lbd*err_z;
  Xadpi1p = Xadpim1 + mrac_gma*s*(Uim1- ...
      Xinsi(INS_ZD) * abs(Xinsi(INS_ZD)) * fdm_Cd);

endfunction


function [Xrefi, Ui, Xadpi, feed_fwd] = ctl_mrac_run(Xinsi, Xrefim1, Xspi, dt, Xadpim1, Uim1)
  // adapt model parameters
  [Xadpi] = ctl_mrac(Xadpim1, Xinsi, Xrefim1, Uim1);
  // update reference
  Xrefi = ctl_update_ref(Xrefim1, Xspi, dt);
  // error control
  err_z  = Xinsi(INS_Z) - Xrefi(REF_Z);
  err_z  = trim(err_z, -ctl_max_err_z, ctl_max_err_z);
  err_zd = Xinsi(INS_ZD) - Xrefi(REF_ZD);
  // invert model
  if 1
    feed_fwd = ( Xrefi(REF_ZDD) + fdm_g ) / Xadpi(1);
  else
//    feed_fwd = ( Xrefi(REF_ZDD) + fdm_g ) / Xadpi + ...
//	Xinsi(INS_ZD) * abs(Xinsi(INS_ZD)) * fdm_Cd;
  feed_fwd = ( Xrefi(REF_ZDD) + fdm_g ) / Xadpi(1) + ...
      Xrefi(REF_ZD) * abs(Xrefi(REF_ZD)) * fdm_Cd;
   end
  // compute command
  feed_bck = 1/Xadpi(1)*(ctl_kp * err_z + ctl_kd * err_zd);
  Ui = feed_fwd + feed_bck;
  Ui = trim(Ui, ctl_min_thrust, ctl_max_thrust);
endfunction
