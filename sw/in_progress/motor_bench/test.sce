clear();
getf('mb_utils.sci');

//filename = "mb_log.txt";
args = sciargs();
[nb_args, foo] = size(args)
filename = args(nb_args);


[time, throttle, rpm, amp, thrust, torque] = read_mb_log(filename);


f_sample = 200.;
fc = 100.;
f_rpm = low_pass_filter(f_sample, fc, rpm);


xbasc();
subplot(3,1,1)
xtitle('Throttle');
plot2d(time, throttle);

subplot(3,1,2)
xtitle('Rpm');
plot2d(time, rpm);

subplot(3,1,3)
xtitle('Filtered Rpm');
plot2d(time, f_rpm);
//plot2d(rpm, throttle);
