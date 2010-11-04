clear();

flt1 = 'flq';
%flt2 = 'ice';
flt2 = 'fcr';

filename1 = ["out_" flt1 ".txt"];
filename2 = ["out_" flt2 ".txt"];

trim_log=true;%false;
time_start=7;
time_stop=25;

plot_true_state = true;
true_state_filename="/home/poine/work/savannah/paparazzi3.broken_stm32_deletion/trunk/sw/simulator/scilab/q6d/data/stop_stop_fdm.txt"

true_dat = load(true_state_filename);
dat1 = load(filename1);
dat2 = load(filename2);
% extract the part of interrest
if (trim_log)
  idx_roi=find(dat1(:,1)>time_start & dat1(:,1)<time_stop);
  dat1 = dat1(idx_roi,:);
  idx_roi=find(dat2(:,1)>time_start & dat2(:,1)<time_stop);
  dat2 = dat2(idx_roi,:);
   if plot_true_state
     idx_roi=find(true_dat(:,1)>time_start & true_dat(:,1)<time_stop);
     true_dat = true_dat(idx_roi,:);
   end
  end

% name fields
time1=dat1(:,1);
quat1=dat1(:,2:5);
%rate1=dat1(:,6:8);
bias1=dat1(:,9:11);
%P1=dat1(:,12:17);

time2=dat2(:,1);
quat2=dat2(:,2:5);
%rate2=dat2(:,6:8);
bias2=dat2(:,9:11);
%P2=dat2(:,12:17);

% computes euler angles
eulers1 = eulers_of_quat(quat1);
eulers2 = eulers_of_quat(quat2);
if plot_true_state
  eulers_true = eulers_of_quat(true_dat(:,2:5));
  eulers_true(:,3) = unwrap(eulers_true(:,3));
end

eulers1(:,3) = unwrap(eulers1(:,3));
eulers2(:,3) = unwrap(eulers2(:,3));

figure(1);
clf();
subplot(3,1,1);
hold on
plot(time1, deg_of_rad(eulers1(:,1)), 'r');
plot(time2, deg_of_rad(eulers2(:,1)), 'b');
if plot_true_state
  plot(true_dat(:,1), deg_of_rad(eulers_true(:,1)), 'g');
  legend(flt1, flt2, 'true');
else
legend(flt1, flt2);	
end
plot(time2, zeros(length(time2), 1), 'k');
title('phi');



subplot(3,1,2);
hold on
plot(time1, deg_of_rad(eulers1(:,2)), 'r');
plot(time2, deg_of_rad(eulers2(:,2)), 'b');
if plot_true_state
  plot(true_dat(:,1), deg_of_rad(eulers_true(:,2)), 'g');
  legend(flt1, flt2, 'true');
else
legend(flt1, flt2);	
end
plot(time2, zeros(length(time2), 1), 'k');
title('theta');

subplot(3,1,3);
hold on
plot(time1, deg_of_rad(eulers1(:,3)), 'r');
plot(time2, deg_of_rad(eulers2(:,3)), 'b');
if plot_true_state
  plot(true_dat(:,1), deg_of_rad(eulers_true(:,3)), 'g');
  legend(flt1, flt2, 'true');
else
legend(flt1, flt2);	
end
title('psi');


figure(2);
clf();
subplot(3,1,1);
hold on
plot(time1, deg_of_rad(bias1(:,1)), 'r');
plot(time2, deg_of_rad(bias2(:,1)), 'b');
plot(time2, zeros(length(time2), 1), 'k');
title('bp');
legend(flt1, flt2);

subplot(3,1,2);
hold on
plot(time1, deg_of_rad(bias1(:,2)), 'r');
plot(time2, deg_of_rad(bias2(:,2)), 'b');
plot(time2, zeros(length(time2), 1), 'k');
title('bq');
legend(flt1, flt2);

subplot(3,1,3);
hold on
plot(time1, deg_of_rad(bias1(:,3)), 'r');
plot(time2, deg_of_rad(bias2(:,3)), 'b');
title('br');
legend(flt1, flt2);