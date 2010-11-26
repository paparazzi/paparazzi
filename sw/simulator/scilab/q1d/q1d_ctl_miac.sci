//
//  Adaptation of the invert dynamic model parameters :
//  dimension one kalman filter estimating invert
//  of the mass
//
adp_sys_noise = 0.0005;
adp_accel_noise = 0.5;

adp_min_cmd = 0.01;

MIAC_USE_VNULL = 1;
miac_vnull = 7.;


function [Xadpi1p, Padpi1p, Madpi1] = ctl_adapt_model(Xadpip, Padpip, Xinsi, Ui)

  if (Ui < adp_min_cmd)
    Xadpi1p = Xadpip
    // so propagate covariance only
    Padpi1p = Padpip; // + adp_sys_noise;
    Madpi1 = 0;
  else
    // Propagate
    // we're estimating a constant
    Xadpi1m = Xadpip
    // so propagate covariance only
    Padpi1m = Padpip + adp_sys_noise;

    // Update
    // our measurement
    Madpi1 = (Xinsi(INS_ZDD) + fdm_g) / ( Ui );
    if MIAC_USE_VNULL
      Madpi1 = Madpi1 / (1. - Xinsi(INS_ZD)/miac_vnull);
    end
    // compute residual
    res =  Madpi1 - Xadpi1m;
    // kalman gain
    adp_meas_noise = adp_accel_noise/Ui;
    E = Padpi1m + adp_meas_noise;
    K = Padpi1m / E;
    // update state and covariance
    Padpi1p = Padpi1m - K * Padpi1m;
    Xadpi1p = Xadpi1m + K * res;
  end

endfunction


function [Xrefi, Ui, Xadpi, Padpi, Madpi, feed_fwd] = ctl_miac_run(Xinsi, Xrefim1, Xspi, dt, Xadpim1, Padpim1, Uim1)
  // adapt model parameters
  [Xadpi, Padpi, Madpi] = ctl_adapt_model(Xadpim1, Padpim1, Xinsi, Uim1)
  // update reference
  Xrefi = ctl_update_ref(Xrefim1, Xspi, dt);
  // error control
  err_z  = Xinsi(INS_Z) - Xrefi(REF_Z);
  err_z  = trim(err_z, -ctl_max_err_z, ctl_max_err_z);
  err_zd = Xinsi(INS_ZD) - Xrefi(REF_ZD);
  // invert model
  feed_fwd = ( Xrefi(REF_ZDD) + fdm_g ) / Xadpi(1);
  if MIAC_USE_VNULL
    feed_fwd  = feed_fwd / (1 -  Xrefi(REF_ZD) / miac_vnull);
  end
  // compute command
  feed_bck = ctl_kp * err_z + ctl_kd * err_zd
  feed_fwd = trim(feed_fwd, ctl_min_cmd, ctl_max_cmd);
  Ui = feed_fwd + feed_bck;
  Ui = trim(Ui, ctl_min_cmd, ctl_max_cmd);

endfunction

