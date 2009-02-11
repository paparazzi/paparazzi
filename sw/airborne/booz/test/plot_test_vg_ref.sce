clear();

M=fscanfMat('traj.out');

time    = M(:,1);
fsp     = M(:,2);
fz      = M(:,3);
fzd     = M(:,4);
fzdd    = M(:,5);
iz      = M(:,6);
izd     = M(:,7);
izdd    = M(:,8);


clf();

drawlater();

subplot(3,1,1);
xtitle('z', 'time (s)','');
plot2d(time, fsp, 1);
plot2d(time, fz, 2);
plot2d(time, iz, 3);
legends(["sp", "float", "int"],[1 2 3], with_box=%f, opt="ur");

subplot(3,1,2);
xtitle('zd', 'time (s)','');
plot2d(time, fzd, 2);
plot2d(time, izd, 3);
legends(["float", "int"],[2 3], with_box=%f, opt="lr");

subplot(3,1,3);
xtitle('zdd', 'time (s)','');
plot2d(time, fzdd, 2);
plot2d(time, izdd, 3);
legends(["float", "int"],[2 3], with_box=%f, opt="lr");

drawnow();
