clear();

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
t2 = 8.;
dt = 1/512;
time1 = t0:dt:t1;
time2 = t1:dt:t2;

// trajectory generation
if 1
  dyn = [rad_of_deg(500) 0.7; rad_of_deg(500) 0.7];
  max_speed = [ 5                        2.5];
  max_accel = [ 9.81*tan(rad_of_deg(30)) 0.5*9.81];
  b0 = [-5  0];
  b1 = [ 5 -5];
  b2 = [ 0 -5];
  [fo_traj1] = sbb_gen_traj(time1, dyn, max_speed, max_accel, b0, b1);
  b1 = [fo_traj1(1,1,$) fo_traj1(2,1,$)];
  [fo_traj2] = sbb_gen_traj(time2, dyn, max_speed, max_accel, b1, b2);
  [time, fo_traj] = merge_traj(list(time1, time2), list(fo_traj1, fo_traj2));
else
  [fo_traj] = fo_traj_swing(time1);
  time = time1;
end

fdm_init(time, df_state_of_fo(fo_traj(:,:,1)), [0.25 0.25]');

ctl_init(time);

diff_flat_cmd = zeros(2,length(time));
diff_flat_ref = zeros(FDM_SSIZE, length(time));
diff_flat_ref(:,1) = df_state_of_fo(fo_traj(:,:,1));

fb_cmd = zeros(2,length(time));
motor_cmd = zeros(2,length(time));

adp_init(time, [15 100]', []);
sensors_init(time)

for i=2:length(time)
//  diff_flat_cmd(:,i) = df_input_of_fo(fo_traj(:,:,i), fdm_Ct0/fdm_mass, fdm_la*fdm_Ct0/fdm_inertia);
  diff_flat_cmd(:,i-1) = df_input_of_fo(fo_traj(:,:,i-1), adp_est(1,i-1), adp_est(2,i-1));
  diff_flat_ref(:,i-1) = df_state_of_fo(fo_traj(:,:,i-1));
  fb_cmd(:,i-1) = ctl_compute_feeback(fdm_state(:,i-1),diff_flat_ref(:,i-1), diff_flat_cmd(:,i-1),  adp_est(1,i-1), adp_est(2,i-1)); 
  global ctl_u;
  ctl_u(:,i-1) = diff_flat_cmd(:,i-1) + fb_cmd(:,i-1);
  MotorsOfCmds = 0.5*[1 -1 ; 1 1];
  motor_cmd(:,i-1) =  MotorsOfCmds * ctl_u(:,i-1);
  fdm_run(i, motor_cmd(:,i-1));
  sensors_run(i);
  adp_run(i);
end

  
set("current_figure",1);
display_control(time, diff_flat_ref, fdm_state, diff_flat_cmd, fb_cmd, motor_cmd );

set("current_figure",2);
display_adaptation();

