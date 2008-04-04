clear();
M = fscanfMat('output.log');
time = M(:,1);
in_float = M(:,2);
out_float = M(:,3);
in_int = M(:,4);
out_int = M(:,5);

clf();
drawlater
subplot(2,1,1)
plot2d(time, in_float, 2);
plot2d(time, out_float, 3);

subplot(2,1,2)
plot2d(time, in_int, 2);
plot2d(time, out_int, 3);
drawnow