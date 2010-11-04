clear();

algebra_common;

filename = "out.txt";
trim_log=false;
time_start=28;
time_stop=80;

dat = load(filename);

% extract the part of interrest
if (trim_log)
  idx_roi=find(dat(:,1)>time_start & dat(:,1)<time_stop);
  dat = dat(idx_roi,:);
end

% name fields
time=dat(:,1);
quat=dat(:,2:5);
rate=dat(:,6:8);
bias=dat(:,9:11);
P=dat(:,12:17);


% computes euler angles
eulers = eulers_of_quat(quat);



figure(1);
clf();
plot(time, rate);
figure(2);
clf();
plot(time, bias);
figure(3);
clf();
plot(time, eulers,'.');
figure(4);
clf();
plot(time, P,'.');