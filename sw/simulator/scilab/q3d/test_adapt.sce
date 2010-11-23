clear();
clearglobal();

exec('q3d_utils.sci');

exec('q3d_sbb.sci');
exec('q3d_fo_traj_misc.sci');

exec('q3d_diff_flatness.sci');
exec('q3d_ctl.sci');
exec('q3d_adaptation.sci');

exec('q3d_fdm.sci');
exec('q3d_sensors.sci');

exec('q3d_display.sci');

t0 = 0;
t1 = 4.;
t2 = 10.;
dt = 1/512;
time1 = t0:dt:t1;
time2 = t1:dt:t2;

// trajectory generation
if 1
  dyn = [rad_of_deg(400) 0.7; rad_of_deg(400) 0.7];
  max_speed = [ 5                        2.5];
  max_accel = [ 9.81*tan(rad_of_deg(30)) 0.5*9.81];
  b0 = [-5  0];
  b1 = [ 5  0];
  b2 = [ 0  0];
  [fo_traj1] = sbb_gen_traj(time1, dyn, max_speed, max_accel, b0, b1);
  b1 = [fo_traj1(1,1,$) fo_traj1(2,1,$)];
  [fo_traj2] = sbb_gen_traj(time2, dyn, max_speed, max_accel, b1, b2);
  [time, fo_traj] = merge_traj(list(time1, time2), list(fo_traj1, fo_traj2));
else
  [fo_traj] = fo_traj_swing(time1);
  time = time1;
end

fdm_init(time, df_state_of_fo(fo_traj(:,:,1)), [0.25 0.25]');

if 0
  global fdm_perturb;
  k=find(time > 5 & time < 5.05);
  fdm_perturb(FDM_AX,k) = 10*ones(1,length(k));
end

ctl_init(time);
adp_init(time, [16.5 110]', []);
sensors_init(time)

for i=2:length(time)
  ctl_run(i-1, adp_est(1,i-1), adp_est(2,i-1));
//  ctl_run(i-1, 16, 100);
  fdm_run(i, ctl_motor_cmd(:,i-1));
  sensors_run(i);
  adp_run(i);
end


set("current_figure",1);
display_control(time, ctl_diff_flat_ref, fdm_state, ctl_diff_flat_cmd, ctl_fb_cmd, ctl_motor_cmd );

set("current_figure",2);
display_adaptation();

