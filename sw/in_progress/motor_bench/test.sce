clear();
getf('mb_utils.sci');

//filename = "mb_log.txt";
args = sciargs();
[nb_args, foo] = size(args)
filename = args(nb_args);


[time, throttle, rpm, amp, thrust, torque] = read_mb_log(filename);


xbasc();
subplot(2,1,1)
xtitle('Throttle');
plot2d(time, throttle);

subplot(2,1,2)
xtitle('Rpm');
plot2d(time, rpm);
