clear();

exec('q6d_sbb.sci');
exec('q6d_diff_flatness.sci');
exec('q6d_algebra.sci');
exec('q6d_display.sci');

t0 = 0;
t1 = 4.;
dt = 1/512;
time = t0:dt:t1;

dyn = [rad_of_deg(500) 0.7; rad_of_deg(500) 0.7];
max_speed = [5 2.5];
max_accel = [ 9.81*tan(rad_of_deg(30)) 0.5*9.81];

b0 = [ 0    0   0];
b1 = [-10   1  -2];
[fo_traj] = sbb_gen_traj(time, dyn, max_speed, max_accel, b0, b1);


set("current_figure",0);
clf();
display_fo_traj(time, fo_traj);


diff_flat_cmd = zeros(4,length(time));
diff_flat_ref = zeros(DF_REF_SIZE, length(time));
for i=1:length(time)
//  diff_flat_cmd(:,i) = df_input_of_fo(fo_traj(:,:,i));
  diff_flat_ref(:,i) = df_state_of_fo(fo_traj(:,:,i));
end

set("current_figure",2);
clf();
display_df_ref(time, diff_flat_ref);
