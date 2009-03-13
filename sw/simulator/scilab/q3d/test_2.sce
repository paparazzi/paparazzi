clear();
clearglobal();


exec('q3d_utils.sci');
exec('q3d_fdm.sci');
exec('q3d_ctl.sci');



fdm_init(0,7.0);
ctl_init();

global ctl_motor;
ctl_motor(:,1) = fdm_mass * fdm_g * [0.5;0.5];

for i=1:length(fdm_time)-1

  fdm_run(i+1, ctl_motor(:,i));
  ctl_run(i+1);
  
end

//set("current_figure",0);
clf();
//f=get("current_figure");
//f.figure_name="CTL";

if 1
  drawlater();
  ctl_display();
  drawnow();
end
pause
gen_video();
