clear();
clearglobal();


exec('q3d_utils.sci');
exec('q3d_fdm.sci');
exec('q3d_ctl.sci');



fdm_init(0,17.0);
ctl_init();

global ctl_motor;
ctl_motor(:,1) = fdm_mass * fdm_g * [0.5;0.5];

for i=1:length(fdm_time)-1

  fdm_run(i+1, ctl_motor(:,i));
  
  if fdm_time(i+1) < 1
    sp_pos= [ 0; 0];
  elseif fdm_time(i+1) < 5
    sp_pos= [ 1; 1];
  elseif fdm_time(i+1) < 9
    sp_pos= [ 0; 1];
  elseif fdm_time(i+1) < 13
    sp_pos= [ 1; 0];
  elseif fdm_time(i+1) < 17
    sp_pos= [ 0; 0];
  end
  
  ctl_run(i+1, sp_pos);
  
end

//set("current_figure",0);
clf();
//f=get("current_figure");
//f.figure_name="CTL";

if 0
  drawlater();
  ctl_display();
  drawnow();
  pause
end
gen_video();
