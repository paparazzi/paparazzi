clear();
M = fscanfMat('output.log');
time = M(:,1);
in = M(:,2);
out = M(:,3);
clf();
plot2d(time, in, 2);
plot2d(time, out, 3);