clear()

// a0 + a1(t-t0) + ... + an(t-tO)^n






exec('q3d_utils.sci');
exec('q3d_polynomials.sci');
exec('q3d_diff_flatness.sci');
exec('q3d_fdm.sci');
exec('q3d_display.sci');

b0 = [0 0 0 0 0];
b1 = [2 0 0 0 0];
t0 = 0;
t1 = 2;
dt = 0.01;
time = t0:dt:t1;

[coefs] = poly_get_coef_from_bound(time, b0, b1);
[fo_traj] = poly_gen_traj(time, coefs);



set("current_figure",0);
clf();
f=get("current_figure");
f.figure_name="Flat Outputs Trajectory";
poly_display_traj(time, fo_traj);

