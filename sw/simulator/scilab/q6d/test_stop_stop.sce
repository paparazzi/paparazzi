clear();

exec('q6d_sbb.sci');
exec('q6d_diff_flatness.sci');
exec('q6d_fdm.sci');
exec('q6d_algebra.sci');
exec('q6d_display.sci');

t0 = 0;
t1 = 4.;
dt = 1/512;
time = t0:dt:t1;

dyn = [rad_of_deg(500) 0.7; rad_of_deg(500) 0.7];
max_speed = [5 2.5];
max_accel = [ 9.81*tan(rad_of_deg(30)) 0.5*9.81];

//b0 = [ 0    0   0];
//b1 = [-10   1  -2];
b0 = [ 0   0   0 ];
b1 = [ 1   0   -5];
[fo_traj] = sbb_gen_traj(time, dyn, max_speed, max_accel, b0, b1);


set("current_figure",0);
clf();
display_fo_traj(time, fo_traj);


diff_flat_cmd = zeros(4,length(time));
diff_flat_ref = zeros(DF_REF_SIZE, length(time));
for i=1:length(time)
  diff_flat_cmd(:,i) = df_input_of_fo(fo_traj(:,:,i));
  diff_flat_ref(:,i) = df_state_of_fo(fo_traj(:,:,i));
end

set("current_figure",1);
clf();
display_df_ref(time, diff_flat_ref);

set("current_figure",2);
clf();
display_df_cmd(time, diff_flat_cmd)


motor_of_cmd = [ 0.25    0.     0.5   -0.25  
                 0.25   -0.5    0.     0.25  
                 0.25    0.    -0.5   -0.25  
                 0.25    0.5    0.     0.25  ];

motor_cmd = zeros(4,length(time));	    

for i=1:length(time)
  motor_cmd(:,i) = 1/fdm_Ct0 * motor_of_cmd * diff_flat_cmd(:,i);
end

display_motor_cmd(time, motor_cmd);

fdm_init(time);
for i=2:length(time)
  fdm_run(i, motor_cmd(:,i-1));
end

set("current_figure",3);
clf();
//display_fdm(time, fdm_state, fdm_euler)
display_control(time, fdm_state, fdm_euler, diff_flat_ref);