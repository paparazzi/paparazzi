


function display_fdm()

  clf();
  global fdm_time;
  global fdm_state;

  subplot(2,3,1);
  plot2d(fdm_time, fdm_state(FDM_SX, :));
  xtitle('X');

  subplot(2,3,2);
  plot2d(fdm_time, fdm_state(FDM_SZ, :));
  xtitle('Z');

  subplot(2,3,3);
  plot2d(fdm_time, deg_of_rad(fdm_state(FDM_STHETA, :)));
  xtitle('Theta');

  subplot(2,3,4);
  plot2d(fdm_time, fdm_state(FDM_SXD, :));
  xtitle('Xd');

  subplot(2,3,5);
  plot2d(fdm_time, fdm_state(FDM_SZD, :));
  xtitle('Zd');

  subplot(2,3,6);
  plot2d(fdm_time, deg_of_rad(fdm_state(FDM_STHETAD, :)));
  xtitle('Thetad');

endfunction

function display_fo(time, fo)

  clf();

  subplot(5,2,1);
  plot2d(time, matrix(fo(1,1,:), 1, length(time)));
  xtitle('X(0)');

  subplot(5,2,2);
  plot2d(time, matrix(fo(2,1,:), 1, length(time)));
  xtitle('Z(0)');

  subplot(5,2,3);
  plot2d(time, matrix(fo(1,2,:), 1, length(time)));
  xtitle('X(1)');

  subplot(5,2,4);
  plot2d(time, matrix(fo(2,2,:), 1, length(time)));
  xtitle('Z(1)');

  subplot(5,2,5);
  plot2d(time, matrix(fo(1,3,:), 1, length(time)));
  xtitle('X(2)');

  subplot(5,2,6);
  plot2d(time, matrix(fo(2,3,:), 1, length(time)));
  xtitle('Z(2)');

  subplot(5,2,7);
  plot2d(time, matrix(fo(1,4,:), 1, length(time)));
  xtitle('X(3)');

  subplot(5,2,8);
  plot2d(time, matrix(fo(2,4,:), 1, length(time)));
  xtitle('Z(3)');

  subplot(5,2,9);
  plot2d(time, matrix(fo(1,5,:), 1, length(time)));
  xtitle('X(4)');

  subplot(5,2,10);
  plot2d(time, matrix(fo(2,5,:), 1, length(time)));
  xtitle('Z(4)');

endfunction


function display_commands(time, diff_flat_cmd)
  subplot(2,2,1);
  plot2d(time, diff_flat_cmd(1,:));
  xtitle('u1');

  subplot(2,2,2);
  plot2d(time, diff_flat_cmd(2,:));
  xtitle('u2');

  subplot(2,2,3);
  plot2d(time, diff_flat_cmd(1,:)-diff_flat_cmd(2,:));
  xtitle('F1');

  subplot(2,2,4);
  plot2d(time, diff_flat_cmd(1,:)+diff_flat_cmd(2,:));
  xtitle('F2');


endfunction


function display_fo_ref(time, diff_flat_ref)

  subplot(2,3,1);
  plot2d(time, diff_flat_ref(FDM_SX, :));
  xtitle('X');

  subplot(2,3,2);
  plot2d(time, diff_flat_ref(FDM_SZ, :));
  xtitle('Z');

  subplot(2,3,3);
  plot2d(time, deg_of_rad(diff_flat_ref(FDM_STHETA, :)));
  xtitle('Theta');

  subplot(2,3,4);
  plot2d(time, diff_flat_ref(FDM_SXD, :));
  xtitle('Xd');

  subplot(2,3,5);
  plot2d(time, diff_flat_ref(FDM_SZD, :));
  xtitle('Zd');

  subplot(2,3,6);
  plot2d(time, deg_of_rad(diff_flat_ref(FDM_STHETAD, :)));
  xtitle('Thetad');

endfunction

function display_control(time, diff_flat_ref, fdm_state, diff_flat_cmd, fb_cmd, motor_cmd )

  f=get("current_figure");
  f.figure_name="Control";

  clf();

  subplot(4,3,1);
  plot2d(time(2:$-1), ctl_diff_flat_ref(FDM_SX, 2:$-1), 3);
  plot2d(time(2:$-1), fdm_state(FDM_SX, 2:$-1), 2);
  legends(["fdm", "ref"],[2 3], with_box=%f, opt="ul");
  xtitle('X');

  subplot(4,3,2);
  plot2d(time(2:$-1), ctl_diff_flat_ref(FDM_SZ, 2:$-1), 3);
  plot2d(time(2:$-1), fdm_state(FDM_SZ, 2:$-1), 2);
  xtitle('Z');

  subplot(4,3,3);
  plot2d(time(2:$-1), deg_of_rad(ctl_diff_flat_ref(FDM_STHETA, 2:$-1)), 3);
  plot2d(time(2:$-1), deg_of_rad(fdm_state(FDM_STHETA, 2:$-1)), 2);
  xtitle('Theta');

  subplot(4,3,4);
  plot2d(time(2:$-1), ctl_diff_flat_ref(FDM_SXD, 2:$-1), 3);
  plot2d(time(2:$-1), fdm_state(FDM_SXD, 2:$-1), 2);
  xtitle('Xd');

  subplot(4,3,5);
  plot2d(time(2:$-1), diff_flat_ref(FDM_SZD, 2:$-1), 3);
  plot2d(time(2:$-1), fdm_state(FDM_SZD, 2:$-1), 2);
  xtitle('Zd');

  subplot(4,3,6);
  plot2d(time(2:$-1), deg_of_rad(diff_flat_ref(FDM_STHETAD, 2:$-1)), 3);
  plot2d(time(2:$-1), deg_of_rad(fdm_state(FDM_STHETAD, 2:$-1)), 2);
  xtitle('Thetad');


  subplot(4,3,7);
  xset("color",5);
  xfpoly([time(2:$-1) time($-1:-1:2)], [diff_flat_cmd(1,2:$-1) ctl_u(1,$-1:-1:2)]);
  xset("color",1);
  plot2d(time(2:$-1), ctl_u(1,2:$-1), 5);
  plot2d(time(2:$-1), diff_flat_cmd(1,2:$-1), 2);
  xtitle('u_t');

  subplot(4,3,8);
  xset("color",5);
  xfpoly([time(2:$-1) time($-1:-1:2)], [diff_flat_cmd(2,2:$-1) ctl_u(2,$-1:-1:2)]);
  xset("color",1);
  plot2d(time(2:$-1), ctl_u(2,2:$-1), 5);
  plot2d(time(2:$-1), diff_flat_cmd(2,2:$-1), 2);
  xtitle('u_d');

  subplot(4,3,10);
  plot2d(time(2:$-1), motor_cmd(1,2:$-1), 2);
  xtitle('u1');

  subplot(4,3,11);
  plot2d(time(2:$-1), motor_cmd(2,2:$-1), 2);
  xtitle('u2');


endfunction

function display_adaptation()

  f=get("current_figure");
  f.figure_name="Adaptation";

  clf();

  subplot(2,3,1);
  plot2d(fdm_time, adp_y(1,:), 3);
//  plot2d(fdm_time, fdm_Ct0/fdm_mass*ctl_u(CTL_UT,:), 2);
  legends(["y1", "ut"],[3 2], with_box=%f, opt="ul");
  xtitle('apd_y1');

  subplot(2,3,2);
//  plot2d(fdm_time, fdm_Ct0/fdm_mass*ones(1,length(time)),3);
  plot2d(fdm_time, adp_est(ADP_EST_A,:), 2);
  xtitle('adp_est A');

  subplot(2,3,3);
  plot2d(fdm_time, matrix(adp_P(1,1,:), 1, length(time)), 2);
  xtitle('adp_P11');


  subplot(2,3,4);
  plot2d(fdm_time, adp_y(2,:), 3);
//  plot2d(fdm_time, fdm_la*fdm_Ct0/fdm_inertia*adp_ud_f, 2);
  legends(["y2", "udf"],[3 2], with_box=%f, opt="ul");
  xtitle('ud_f');

  subplot(2,3,5);
//  plot2d(fdm_time, fdm_la*fdm_Ct0/fdm_inertia*ones(1, length(time)),3);
  plot2d(fdm_time, adp_est(ADP_EST_B,:), 2);
  xtitle('adp_est B');

  subplot(2,3,6);
  plot2d(fdm_time, matrix(adp_P(2,2,:), 1, length(time)), 2);
  xtitle('adp_P22');

endfunction
