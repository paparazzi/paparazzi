clear();
clearglobal();


exec('q3d_utils.sci');
exec('q3d_fdm.sci');
exec('q3d_ctl.sci');



fdm_init(0,20.);
ctl_init();

for i=1:length(fdm_time)-1
  fdm_run(i+1, [1.227; 1.227]);
  ctl_run(i+1);
  
  
end

drawlater();
set("current_figure",0);
clf();
f=get("current_figure");
f.figure_name="CTL";
ctl_display();
drawnow();