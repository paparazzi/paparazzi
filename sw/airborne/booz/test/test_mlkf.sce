clear();
clearglobal();


exec('../../../simulator/scilab/q6d/q6d_fdm.sci');
exec('../../../simulator/scilab/q6d/q6d_algebra.sci');

global ahrs_quat;
global ahrs_omega;
global ahrs_bias;
global ahrs_cov;

function read_ahrs_output(filename)
  global ahrs_quat;
  global ahrs_omega;
  global ahrs_bias;
  global ahrs_cov;
  ahrs_quat  = zeros(4,length(time));
  ahrs_omega = zeros(3,length(time));
  ahrs_bias  = zeros(3,length(time));
  ahrs_cov   = zeros(6,length(time));
  i=1;
  u=mopen(filename, 'r');
  while meof(u) == 0 & i <= length(time)
    line = mgetl(u, 1);
    if line ~= []
      [nb_scan, t, qi, qx, qy, qz, p, q, r, bp, bq, br, c11, c22, c33, c44, c55, c66] = ...
	  msscanf(1, line, '%f [%f %f %f %f] [%f %f %f] [%f %f %f] [%f %f %f %f %f %f]');
      if nb_scan == 17
	ahrs_quat(:,i)  = [qi qx qy qz]';
	ahrs_omega(:,i) = [p q r]';
	ahrs_bias(:,i)  = [bp bq br]';
	ahrs_cov(:,i)   = [c11 c22 c33 c44 c55 c66]';
	i = i+1;
      end
    end
  end
  mclose(u);
endfunction

function display_ahrs_state()
  clf();
  f=get("current_figure");
  f.figure_name="AHRS state";

  eul_fdm  = zeros(3,length(time));
  eul_ahrs = zeros(3,length(time));
  
  for i=1:length(time)
    eul_fdm(:,i)  = euler_of_quat(fdm_state(FDM_SQI:FDM_SQZ,i));
    eul_ahrs(:,i) = euler_of_quat(ahrs_quat(:,i));
  end
  
  subplot(4,3,1);
  plot2d(time, deg_of_rad(eul_ahrs(1,:)), 3);
  plot2d(time, deg_of_rad(eul_fdm(1,:)), 2);
  legends(["fdm", "ahrs_c"],[2 3], with_box=%f, opt="ul"); 
  xtitle('Phi');

  subplot(4,3,2);
  plot2d(time, deg_of_rad(eul_ahrs(2,:)), 3);
  plot2d(time, deg_of_rad(eul_fdm(2,:)), 2);
  legends(["fdm", "ahrs_c"],[2 3], with_box=%f, opt="ul"); 
  xtitle('Theta');
  
  subplot(4,3,3);
  plot2d(time, deg_of_rad(eul_ahrs(3,:)), 3);
  plot2d(time, deg_of_rad(eul_fdm(3,:)), 2);
  legends(["fdm", "ahrs_c"],[2 3], with_box=%f, opt="ul"); 
  xtitle('Psi');
  
  subplot(4,3,4);
  plot2d(time, deg_of_rad(ahrs_omega(1,:)), 3);
  plot2d(time, deg_of_rad(fdm_state(FDM_SP,:)), 2);
  xtitle('p');
  
  subplot(4,3,5);
  plot2d(time, deg_of_rad(ahrs_omega(2,:)), 3);
  plot2d(time, deg_of_rad(fdm_state(FDM_SQ,:)), 2);
  xtitle('q');

  subplot(4,3,6);
  plot2d(time, deg_of_rad(ahrs_omega(3,:)), 3);
  plot2d(time, deg_of_rad(fdm_state(FDM_SR,:)), 2);
  xtitle('r');

  subplot(4,3,7);
  plot2d(time, deg_of_rad(ahrs_bias(1,:)), 3);
  xtitle('bp');
  
  subplot(4,3,8);
  plot2d(time, deg_of_rad(ahrs_bias(2,:)), 3);
  xtitle('bq');

  subplot(4,3,9);
  plot2d(time, deg_of_rad(ahrs_bias(3,:)), 3);
  xtitle('br');
 
  subplot(4,3,10);

  subplot(4,3,11);
  plot2d(time, ahrs_cov(1,:),3);
  plot2d(time, ahrs_cov(2,:),3);
  plot2d(time, ahrs_cov(3,:),3);
  
  legends(["ahrs_c", "ahrs_s"],[3 4], with_box=%f, opt="ul"); 
  xtitle('cov angles'); 
  
  subplot(4,3,12);
  plot2d(time, ahrs_cov(4,:),3);
  plot2d(time, ahrs_cov(5,:),3);
  plot2d(time, ahrs_cov(6,:),3);
 
  xtitle('cov bias'); 
  
endfunction

load('../../../simulator/scilab/q6d/data/stop_stop_state_sensors.dat', 'time', 'fdm_state', 'sensor_gyro', 'sensor_accel', 'sensor_mag');
read_ahrs_output('out.txt');

display_ahrs_state();