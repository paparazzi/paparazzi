clear();

M=fscanfMat('traj.out');

time    = M(:,1);
fzsp    = M(:,2);
fzdsp   = M(:,3);
fz      = M(:,4);
fzd     = M(:,5);
fzdd    = M(:,6);
iz      = M(:,7);
izd     = M(:,8);
izdd    = M(:,9);


clf();

drawlater();

subplot(3,1,1);
xtitle('z', 'time (s)','');
plot2d(time, fzsp, 1);
plot2d(time, fz, 2);
plot2d(time, iz, 3);
legends(["sp", "float", "int"],[1 2 3], with_box=%f, opt="ur");

subplot(3,1,2);
xtitle('zd', 'time (s)','');
plot2d(time, fzdsp, 1);
plot2d(time, fzd, 2);
plot2d(time, izd, 3);
legends(["float", "int"],[2 3], with_box=%f, opt="lr");

subplot(3,1,3);
xtitle('zdd', 'time (s)','');
plot2d(time, fzdd, 2);
plot2d(time, izdd, 3);
legends(["float", "int"],[2 3], with_box=%f, opt="lr");

drawnow();
