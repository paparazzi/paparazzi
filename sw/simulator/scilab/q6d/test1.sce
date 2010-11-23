clear();
clearglobal();

exec('q6d_algebra.sci');
exec('q6d_fdm.sci');
exec('q6d_sensors.sci');
exec('q6d_ahrs.sci');
exec('q6d_ins.sci');
exec('q6d_guidance.sci');
exec('q6d_stabilization.sci');




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
//  guidance_run(i+1);

  if (fdm_time(i)>1 & fdm_time(i)<2)
    stabilization_sp_quat(:,i+1) = quat_of_euler([rad_of_deg(30) 0 0]');
  else
    stabilization_sp_quat(:,i+1) = quat_null();
  end
  stabilization_sp_thrust(i+1) = guidance_mass / guidance_Ct0 * 9.81;

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
