clear();
getf('rotations.sci');
getf('imu.sci');


rand('seed', 0);
getf('quadrotor.sci');
true_euler0 = [ 0.0; 0.0; 0.0]; 
dt =  0.015625;

[time, true_rates, true_eulers] = quadrotor_gen_roll_step(true_euler0, dt);
[accel, mag, gyro] = imu_sim_misaligned(time, true_rates, true_eulers);




xbasc();
subplot(3,1,1)
plot2d([time]', accel', style=[5, 3, 2], leg="a_x@a_y@a_z");
xtitle( 'Accel', 's', 'volts') ;


true_rates_deg = deg_of_rad(true_rates);
subplot(3,1,2)
plot2d([time]', true_rates_deg', style=[5, 3, 2], leg="rate_p@rate_q@rate_r");
xtitle( 'True rates', 's', 'deg/s') ;

subplot(3,1,3)
plot2d([time]', gyro', style=[5, 3, 2], leg="g_x@g_y@g_z");
xtitle( 'Raw gyros', 's', 'adc') ;




