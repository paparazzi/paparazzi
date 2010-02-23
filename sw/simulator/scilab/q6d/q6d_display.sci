
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


