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

k = find(time > 1110 & time < 1168);


clf();

drawlater();

subplot(3,2,1);
xtitle('X', 'time (s)','');
plot2d(time(k), ffm(k), 3);
plot2d(time(k), ffX(k), 1);

subplot(3,2,2);
xtitle('P', 'time (s)','');
plot2d(time(k), ffP(k), 2);

subplot(3,2,3);
xtitle('X', 'time (s)','');
plot2d(time(k), ifX(k), 1);

subplot(3,2,4);
xtitle('P', 'time (s)','');
plot2d(time(k), ifP(k), 2);

subplot(3,2,5);
xtitle('Zdd', 'time (s)','');
plot2d(time(k), meas(k)/2^10, 1);

subplot(3,2,6);
xtitle('Thrust', 'time (s)','');
plot2d(time(k), thrust(k), 2);
mg = 9.81./ffX(k); 
plot2d(time(k), mg, 3);
mzdd = zdd_ref(k)./ffX(k)./2^10; 
plot2d(time(k), mg - mzdd, 5);

drawnow();
