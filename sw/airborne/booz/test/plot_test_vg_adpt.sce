clear();

M=fscanfMat('traj.out');

time    = M(:,1);
meas    = M(:,2);
thrust  = M(:,3);
zdd_ref = M(:,4);
ffX     = M(:,5);
ffP     = M(:,6);
ffm     = M(:,7);
ifX     = M(:,8);
ifP     = M(:,9);

//k = find(time > 500 & time < 550);
//k = find(time > 1110 & time < 1168);
k = 1:length(time);


clf();

drawlater();

subplot(3,2,1);
xtitle('X', 'time (s)','');
plot2d(time(k), ffm(k), 4);
plot2d(time(k), ffX(k), 3);
plot2d(time(k), ifX(k)/2^18, 5);
legends(["float", "int", "meas"],[3 5 4], with_box=%f, opt="lr");

subplot(3,2,2);
xtitle('P', 'time (s)','');
plot2d(time(k), ffP(k), 3);
plot2d(time(k), ifP(k)/2^18, 5);
legends(["float", "int"],[3 5], with_box=%f, opt="lr");

subplot(3,2,3);
xtitle('X', 'time (s)','');
plot2d(time(k), ffX(k)*2^18, 3);
plot2d(time(k), ifX(k), 5);
legends(["float", "int"],[3 5], with_box=%f, opt="lr");

subplot(3,2,4);
xtitle('P', 'time (s)','');
plot2d(time(k), ffP(k)*2^18, 3);
plot2d(time(k), ifP(k), 5);
legends(["float", "int"],[3 5], with_box=%f, opt="lr");

subplot(3,2,5);
xtitle('Zdd', 'time (s)','');
plot2d(time(k), meas(k)/2^10, 1);

subplot(3,2,6);
xtitle('Thrust', 'time (s)','');

fmg = 9.81./ffX(k); 
img = 9.81./ifX(k)*2^18; 
plot2d(time(k), fmg, 3);
plot2d(time(k), img, 5);

fmzdd = zdd_ref(k)./ffX(k)./2^10; 
imzdd = zdd_ref(k)./ifX(k)./2^10*2^18; 
//plot2d(time(k), fmg - fmzdd, 3);
//plot2d(time(k), img - imzdd, 5);

//plot2d(time(k), thrust(k), 2);

legends(["float", "int"],[3 5], with_box=%f, opt="lr");

drawnow();
