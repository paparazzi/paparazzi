clear();
exec("calibration_utils.sci");

//[raw_mag, raw_accel] = read_log("log_calib_mag_4.dat");
[raw_mag, raw_accel] = read_log("log_calib_mag_b2a2");

//[fraw_mag] = filter_noise(raw_mag,15,300);
fraw_mag = raw_mag;

  
n0 = [ 2000; 2000; 2000];
g0 = [ 4; 4; 4];

[scaled_mag] = scale_sensor(fraw_mag, g0, n0);
[scaled_module] = compute_mod(scaled_mag);


function err = cost_fun(p, z)
  err = (z(1) - p(1))^2 * p(4)^2 + ...
        (z(2) - p(2))^2 * p(5)^2 + ...
        (z(3) - p(3))^2 * p(6)^2 - (2^11)^2;
endfunction
  
[p, err] = datafit(cost_fun, fraw_mag, [n0; g0]);

[scaled_mag2] = scale_sensor(fraw_mag, p(4:6), p(1:3));
[scaled_module2] = compute_mod(scaled_mag2);


gain_foo = [ 2^11/p(4)
             2^11/p(5)
             2^11/p(6) ]

clf();

subplot(4,1,1);
[nl, nc] = size(raw_mag);
plot2d(1:nc, raw_mag(1,:), 1);
plot2d(1:nc, raw_mag(2,:), 2);
plot2d(1:nc, raw_mag(3,:), 3);
xtitle('raw sensors');

subplot(4,1,2);
[nl, nc] = size(fraw_mag);
plot2d(1:nc, scaled_mag(1,:), 1);
plot2d(1:nc, scaled_mag(2,:), 2);
plot2d(1:nc, scaled_mag(3,:), 3);
xtitle('scaled sensors initial guess');

subplot(4,1,3);
plot2d(1:nc, scaled_mag2(1,:), 1);
plot2d(1:nc, scaled_mag2(2,:), 2);
plot2d(1:nc, scaled_mag2(3,:), 3);
xtitle('scaled sensors optimised');

subplot(4,1,4);
plot2d(1:nc, scaled_module, 1);
subplot(4,1,4);
plot2d(1:nc, scaled_module2, 2);
xtitle('norm');
