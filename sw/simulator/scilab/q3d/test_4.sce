clear();
clearglobal();


exec('q3d_string.sci');

[traj1, traj5] = string_get_traj2();
dt = 1/512;
clf();
plot2d(traj1(1,:), traj1(2,:), -1,rect=[-2 -0.5 2 3.5]);
plot2d(traj5(1,:), traj5(2,:), 2,rect=[-2 -0.5 2 3.5]);
