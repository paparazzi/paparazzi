clear();
clearglobal();

exec('q6d_algebra.sci');

exec('q6d_sbb.sci');
exec('q6d_fo_traj_misc.sci');
exec('q6d_diff_flatness.sci');
exec('q6d_ctl.sci');
exec('q6d_sensors.sci');
exec('q6d_fdm.sci');
exec('q6d_display.sci');
exec('q6d_io.sci');


t0 = 0;
t1 = 15.;
t2 = 25.;
dt = 1/512;
time1 = t0:dt:t1;
time2 = t1:dt:t2;

dyn = [rad_of_deg(500) 0.7; rad_of_deg(500) 0.7];
max_speed = [5 2.5];
max_accel = [ 9.81*tan(rad_of_deg(30)) 0.5*9.81];

b0 = [ -5    0   0];
b1 = [ -5    0   0];
b2 = [  5    0  -5];
[fo_traj1] = sbb_gen_traj(time1, dyn, max_speed, max_accel, b0, b1);
b1_t = [fo_traj1(1,1,$) fo_traj1(2,1,$) fo_traj1(3,1,$)];
[fo_traj2] = sbb_gen_traj(time2, dyn, max_speed, max_accel, b1_t, b2);

[time, fo_traj] = merge_traj(list(time1, time2 ), list(fo_traj1, fo_traj2));

set("current_figure",0);
clf();
display_fo_traj(time, fo_traj);

ref0 = df_ref_of_fo(fo_traj(:,:,1));
X0 = [ ref0(DF_REF_X:DF_REF_Z)
       ref0(DF_REF_XD:DF_REF_ZD)
       quat_of_euler([ref0(DF_REF_PHI:DF_REF_PSI,1)])
       ref0(DF_REF_P:DF_REF_R) ];
fdm_init(time, X0);

ctl_init(time);
sensors_init(time);

for i=2:length(time)

  ctl_run(i-1, fo_traj(:,:,i-1));
  fdm_run(i, ctl_motor_cmd(:,i-1));
  sensors_run(i);

end

set("current_figure",1);
clf();
display_sensors(time);

set("current_figure",2);
clf();
display_control(time, fdm_state, fdm_euler, ctl_diff_flat_ref);

io_dump_fdm_sensor_dat(time, 'data/stop_stop_state_sensors');
