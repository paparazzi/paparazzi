clear();
clearglobal();

exec('tins_algebra.sci');
exec('tins_fdm.sci');
exec('tins_sensors.sci');
exec('tins_ahrs.sci');
exec('tins_ins.sci');
exec('tins_guidance.sci');
exec('tins_stabilization.sci');




fdm_init(0.,4.);
sensors_init();
ahrs_init();
ins_init();
stabilization_init();
guidance_init();

sensors_run(1);

for i=1:length(fdm_time)-1

  fdm_run(i+1, stabilization_cmd_motors(:,i));
  sensors_run(i+1);
  ahrs_propagate(i+1);
  ins_propagate(i+1);
  guidance_run(i+1);
  stabilization_run(i+1);
  
end

if 0
  set("current_figure",0);
  clf();
  f=get("current_figure");
  f.figure_name="AHRS";
  drawlater();
  ahrs_display();
  drawnow();
  
  set("current_figure",1);
  clf();
  f=get("current_figure");
  f.figure_name="INS";
  drawlater();
  ins_display();
  drawnow();
end

set("current_figure",2);
clf();
f=get("current_figure");
f.figure_name="STABILIZATION";
drawlater();
stabilization_display();
drawnow();

set("current_figure",3);
clf();
f=get("current_figure");
f.figure_name="GUIDANCE";
drawlater();
guidance_display();
drawnow();
