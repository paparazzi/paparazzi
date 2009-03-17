clear();
clearglobal();


exec('q3d_utils.sci');
exec('q3d_fdm.sci');
exec('q3d_ctl.sci');
exec('q3d_string.sci');

[traj] = string_get_traj();
dt = 1/512;
[nr,nc]=size(traj);  
duration = nc*dt;


fdm_init(0,duration);
ctl_init();

global ctl_motor;
ctl_motor(:,1) = fdm_mass * fdm_g * [0.5;0.5];

for i=1:length(fdm_time)-1

  fdm_run(i+1, ctl_motor(:,i));
//  sp_pos= [ 0; 0];
  sp_pos = traj(:,i);
  ctl_run(i+1, sp_pos);
  
end

//set("current_figure",0);
clf();
//f=get("current_figure");
//f.figure_name="CTL";

if 1
  clf();
  plot2d(traj(1,:), traj(2,:),1);
  plot2d(fdm_state(FDM_SX,:), fdm_state(FDM_SZ,:),2);
  plot2d(ctl_ref_0(AXIS_X,:), ctl_ref_0(AXIS_Z,:),3);
  pause
end

if 1
  clf();
  drawlater();
  ctl_display();
  drawnow();
  pause
end
if 1
gen_video();
end
