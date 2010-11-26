clear()
exec('q3d_utils.sci');

exec('q3d_polynomials.sci');
exec('q3d_fo_traj_misc.sci');

exec('q3d_diff_flatness.sci');
exec('q3d_fdm.sci');
exec('q3d_display.sci');
exec('q3d_povray.sci');


t0 = 0;
t1 = 5.;
t2 = 15.;
t3 = 20.;
dt = 1/512;
time1 = t0:dt:t1;
time2 = t1:dt:t2;
time3 = t2:dt:t3;

[fo_traj2] = fo_traj_circle(time2, [0 0], 2, rad_of_deg(45));

b0 = [-5 0 0 0 0; 0 0 0 0 0];
b1 = [ fo_traj2(1,:,1); fo_traj2(2,:,1)];
[coefs] = poly_get_coef_from_bound(time1, b0, b1);
[fo_traj1] = poly_gen_traj(time1, coefs);

b0 = [ fo_traj2(1,:,$); fo_traj2(2,:,$)];
b1 = [-5 0 0 0 0; 0 0 0 0 0];
[coefs] = poly_get_coef_from_bound(time3, b0, b1);
[fo_traj3] = poly_gen_traj(time3, coefs);


[time, fo_traj] = merge_traj(list(time1, time2, time3), list(fo_traj1, fo_traj2, fo_traj3));


diff_flat_cmd = zeros(2,length(time));
diff_flat_ref = zeros(FDM_SSIZE, length(time));
for i=1:length(time)
  diff_flat_cmd(:,i) = df_input_of_fo(fo_traj(:,:,i));
  diff_flat_ref(:,i) = df_state_of_fo(fo_traj(:,:,i));
end

fdm_init(time, df_state_of_fo(fo_traj(:,:,1)));
for i=2:length(time)
  u1 = diff_flat_cmd(1,i-1);
  u2 = diff_flat_cmd(2,i-1);
  m1 = 0.5*(u1+u2);
  m2 = 0.5*(u1-u2);
  fdm_run(i, [m1 m2]')
end

set("current_figure",0);
clf();
f=get("current_figure");
f.figure_name="Flat Outputs Trajectory";
display_fo(time, fo_traj);

set("current_figure",1);
clf();
f=get("current_figure");
f.figure_name="Commands";
display_commands(time, diff_flat_cmd);

set("current_figure",2);
clf();
f=get("current_figure");
f.figure_name="Reference";
display_fo_ref(time, diff_flat_ref);

set("current_figure",3);
clf();
f=get("current_figure");
f.figure_name="FDM";
display_fdm();


povray_draw(time, diff_flat_ref);
