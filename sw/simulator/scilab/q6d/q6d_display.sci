
function display_fo_traj(time, fo_traj)

  f=get("current_figure");
  f.figure_name="Flat Outputs Trajectory";

  subplot(5,4,1);
  plot2d(time, matrix(fo_traj(1,1,:), 1, length(time)));
  xtitle('X(0)');

  subplot(5,4,2);
  plot2d(time, matrix(fo_traj(2,1,:), 1, length(time)));
  xtitle('Y(0)');

  subplot(5,4,3);
  plot2d(time, matrix(fo_traj(3,1,:), 1, length(time)));
  xtitle('Z(0)');

  subplot(5,4,4);
  plot2d(time, deg_of_rad(matrix(fo_traj(4,1,:), 1, length(time))));
  xtitle('PSI(0)');



  subplot(5,4,5);
  plot2d(time, matrix(fo_traj(1,2,:), 1, length(time)));
  xtitle('X(1)');

  subplot(5,4,6);
  plot2d(time, matrix(fo_traj(2,2,:), 1, length(time)));
  xtitle('Y(1)');

  subplot(5,4,7);
  plot2d(time, matrix(fo_traj(3,2,:), 1, length(time)));
  xtitle('Z(1)');

  subplot(5,4,8);
  plot2d(time, deg_of_rad(matrix(fo_traj(4,2,:), 1, length(time))));
  xtitle('PSI(1)');



  subplot(5,4,9);
  plot2d(time, matrix(fo_traj(1,3,:), 1, length(time)));
  xtitle('X(2)');

  subplot(5,4,10);
  plot2d(time, matrix(fo_traj(2,3,:), 1, length(time)));
  xtitle('Y(2)');

  subplot(5,4,11);
  plot2d(time, matrix(fo_traj(3,3,:), 1, length(time)));
  xtitle('Z(2)');

  subplot(5,4,12);
  plot2d(time, deg_of_rad(matrix(fo_traj(4,3,:), 1, length(time))));
  xtitle('PSI(2)');



  subplot(5,4,13);
  plot2d(time, matrix(fo_traj(1,4,:), 1, length(time)));
  xtitle('X(3)');

  subplot(5,4,14);
  plot2d(time, matrix(fo_traj(2,4,:), 1, length(time)));
  xtitle('Y(3)');

  subplot(5,4,15);
  plot2d(time, matrix(fo_traj(3,4,:), 1, length(time)));
  xtitle('Z(3)');

//  subplot(5,4,16);
//  plot2d(time, deg_of_rad(matrix(fo_traj(4,4,:), 1, length(time))));
//  xtitle('PSI(3)');



  subplot(5,4,17);
  plot2d(time, matrix(fo_traj(1,5,:), 1, length(time)));
  xtitle('X(4)');

  subplot(5,4,18);
  plot2d(time, matrix(fo_traj(2,5,:), 1, length(time)));
  xtitle('Y(4)');

  subplot(5,4,19);
  plot2d(time, matrix(fo_traj(3,5,:), 1, length(time)));
  xtitle('Z(4)');

//  subplot(5,4,20);
//  plot2d(time, deg_of_rad(matrix(fo_traj(4,5,:), 1, length(time))));
//  xtitle('PSI(4)');


endfunction


function display_df_ref(time, diff_flat_ref)

  f=get("current_figure");
  f.figure_name="Reference";

  subplot(6,2,1);
  plot2d(time, diff_flat_ref(DF_REF_X,:));
  xtitle('X');

  subplot(6,2,3);
  plot2d(time, diff_flat_ref(DF_REF_Y,:));
  xtitle('Y');

  subplot(6,2,5);
  plot2d(time, diff_flat_ref(DF_REF_Z,:));
  xtitle('Z');


  subplot(6,2,7);
  plot2d(time, deg_of_rad(diff_flat_ref(DF_REF_PHI,:)));
  xtitle('PHI');

  subplot(6,2,9);
  plot2d(time, deg_of_rad(diff_flat_ref(DF_REF_THETA,:)));
  xtitle('THETA');

  subplot(6,2,11);
  plot2d(time, deg_of_rad(diff_flat_ref(DF_REF_PSI,:)));
  xtitle('PSI');


  subplot(6,2,2);
  plot2d(time, diff_flat_ref(DF_REF_XD,:));
  xtitle('XD');

  subplot(6,2,4);
  plot2d(time, diff_flat_ref(DF_REF_YD,:));
  xtitle('YD');

  subplot(6,2,6);
  plot2d(time, diff_flat_ref(DF_REF_ZD,:));
  xtitle('ZD');


  subplot(6,2,8);
  plot2d(time, deg_of_rad(diff_flat_ref(DF_REF_P,:)));
  xtitle('P');

  subplot(6,2,10);
  plot2d(time, deg_of_rad(diff_flat_ref(DF_REF_Q,:)));
  xtitle('Q');

  subplot(6,2,12);
  plot2d(time, deg_of_rad(diff_flat_ref(DF_REF_R,:)));
  xtitle('R');

endfunction


function display_df_cmd(time, diff_flat_cmd)

  f=get("current_figure");
  f.figure_name="Command";

  subplot(2,4,1);
  plot2d(time, diff_flat_cmd(1,:));
  xtitle('Ut');

  subplot(2,4,2);
  plot2d(time, diff_flat_cmd(2,:));
  xtitle('Up');

  subplot(2,4,3);
  plot2d(time, diff_flat_cmd(3,:));
  xtitle('Uq');

  subplot(2,4,4);
  plot2d(time, diff_flat_cmd(4,:));
  xtitle('Ur');

endfunction


function display_motor_cmd(time, motor_cmd)

  subplot(2,4,5);
  plot2d(time, motor_cmd(1,:));
  xtitle('F1');

  subplot(2,4,6);
  plot2d(time, motor_cmd(2,:));
  xtitle('F2');

  subplot(2,4,7);
  plot2d(time, motor_cmd(3,:));
  xtitle('F3');

  subplot(2,4,8);
  plot2d(time, motor_cmd(4,:));
  xtitle('F4');

endfunction


function display_fdm(time, state, euler)

  f=get("current_figure");
  f.figure_name="FDM";

  subplot(4,3,1);
  plot2d(time, state(FDM_SX,:));
  xtitle('X');

  subplot(4,3,2);
  plot2d(time, state(FDM_SY,:));
  xtitle('Y');

  subplot(4,3,3);
  plot2d(time, state(FDM_SZ,:));
  xtitle('Z');


  subplot(4,3,4);
  plot2d(time, state(FDM_SXD,:));
  xtitle('Xd');

  subplot(4,3,5);
  plot2d(time, state(FDM_SYD,:));
  xtitle('Yd');

  subplot(4,3,6);
  plot2d(time, state(FDM_SZD,:));
  xtitle('Zd');


  subplot(4,3,7);
  plot2d(time, deg_of_rad(euler(FDM_EPHI,:)));
  xtitle('Phi');

  subplot(4,3,8);
  plot2d(time, deg_of_rad(euler(FDM_ETHETA,:)));
  xtitle('Theta');

  subplot(4,3,9);
  plot2d(time, deg_of_rad(euler(FDM_EPSI,:)));
  xtitle('Psi');


  subplot(4,3,10);
  plot2d(time, deg_of_rad(state(FDM_SP,:)));
  xtitle('p');

  subplot(4,3,11);
  plot2d(time, deg_of_rad(state(FDM_SQ,:)));
  xtitle('q');

  subplot(4,3,12);
  plot2d(time, deg_of_rad(state(FDM_SR,:)));
  xtitle('r');


endfunction


function display_control(time, fdm_state, fdm_euler, diff_flat_ref)

  f=get("current_figure");
  f.figure_name="Control";

  subplot(4,3,1);
  plot2d(time, fdm_state(FDM_SX,:), 2);
  plot2d(time, diff_flat_ref(DF_REF_X,:),3);
  xtitle('X');
  legends(["fdm", "ref"],[2 3], with_box=%f, opt="ul");

  subplot(4,3,2);
  plot2d(time, fdm_state(FDM_SY,:), 2);
  plot2d(time, diff_flat_ref(DF_REF_Y,:),3);
  xtitle('Y');

  subplot(4,3,3);
  plot2d(time, fdm_state(FDM_SZ,:), 2);
  plot2d(time, diff_flat_ref(DF_REF_Z,:),3);
  xtitle('Z');


  subplot(4,3,4);
  plot2d(time, fdm_state(FDM_SXD,:), 2);
  plot2d(time, diff_flat_ref(DF_REF_XD,:),3);
  xtitle('Xd');

  subplot(4,3,5);
  plot2d(time, fdm_state(FDM_SYD,:), 2);
  plot2d(time, diff_flat_ref(DF_REF_YD,:),3);
  xtitle('Yd');

  subplot(4,3,6);
  plot2d(time, fdm_state(FDM_SZD,:), 2);
  plot2d(time, diff_flat_ref(DF_REF_ZD,:),3);
  xtitle('Zd');


  subplot(4,3,7);
  plot2d(time, deg_of_rad(fdm_euler(FDM_EPHI,:)), 2);
  plot2d(time, deg_of_rad(diff_flat_ref(DF_REF_PHI,:)), 3);
  xtitle('Phi');

  subplot(4,3,8);
  plot2d(time, deg_of_rad(fdm_euler(FDM_ETHETA,:)), 2);
  plot2d(time, deg_of_rad(diff_flat_ref(DF_REF_THETA,:)), 3);
  xtitle('Theta');

  subplot(4,3,9);
  plot2d(time, deg_of_rad(fdm_euler(FDM_EPSI,:)), 2);
  plot2d(time, deg_of_rad(diff_flat_ref(DF_REF_PSI,:)), 3);
  xtitle('Psi');


  subplot(4,3,10);
  plot2d(time, deg_of_rad(fdm_state(FDM_SP,:)), 2);
  plot2d(time, deg_of_rad(diff_flat_ref(DF_REF_P,:)), 3);
  xtitle('p');

  subplot(4,3,11);
  plot2d(time, deg_of_rad(fdm_state(FDM_SQ,:)), 2);
  plot2d(time, deg_of_rad(diff_flat_ref(DF_REF_Q,:)), 3);
  xtitle('q');

  subplot(4,3,12);
  plot2d(time, deg_of_rad(fdm_state(FDM_SR,:)), 2);
  plot2d(time, deg_of_rad(diff_flat_ref(DF_REF_R,:)), 3);
  xtitle('r');


endfunction


function display_sensors(time)

  f=get("current_figure");
  f.figure_name="Sensors";

  subplot(3,3,1);
  plot2d(time, deg_of_rad(sensor_gyro(1,:)), 2);
  xtitle('gyro p');

  subplot(3,3,2);
  plot2d(time, deg_of_rad(sensor_gyro(2,:)), 2);
  xtitle('gyro q');

  subplot(3,3,3);
  plot2d(time, deg_of_rad(sensor_gyro(3,:)), 2);
  xtitle('gyro r');


  subplot(3,3,4);
  plot2d(time, sensor_accel(1,:), 2);
  xtitle('accel x');

  subplot(3,3,5);
  plot2d(time, sensor_accel(2,:), 2);
  xtitle('accel y');

  subplot(3,3,6);
  plot2d(time, sensor_accel(3,:), 2);
  xtitle('accel z');

  subplot(3,3,7);
  plot2d(time, sensor_mag(1,:), 2);
  xtitle('mag x');

  subplot(3,3,8);
  plot2d(time, sensor_mag(2,:), 2);
  xtitle('mag y');

  subplot(3,3,9);
  plot2d(time, sensor_mag(3,:), 2);
  xtitle('mag z');

endfunction

