//
//
//
//
//



SENSORS_ACCEL = 1;
SENSORS_BARO  = 2;
SENSORS_ACCEL_BIAS = 3;
SENSORS_SIZE  = 3;


accel_noise_std_dev = 0.20;
accel_bias = 0.4;
baro_noise_std_dev = 0.1;
baro_period = 10;
baro_step = 0.06;


function [Xsensorsi] = sensors_run(ti, Xfdmi)
  Xsensorsi = zeros(SENSORS_SIZE, 1);
  Xsensorsi(SENSORS_ACCEL) = Xfdmi(FDM_ZDD) + 9.81 +...
                             accel_bias +...
			     accel_noise_std_dev * rand(1,1,'normal');
  Xsensorsi(SENSORS_ACCEL_BIAS) = accel_bias;

  baro_real = Xfdmi(FDM_Z) + baro_noise_std_dev * rand(1,1,'normal');
  baro_disc = round(baro_real/baro_step) * baro_step;
  Xsensorsi(SENSORS_BARO) = baro_disc;
endfunction

//
// Simple display
//
function sensors_display_simple(Xsensors, time)

nr = 2;
nc = 1;

subplot(nr,nc,1);
plot2d(time, Xsensors(SENSORS_ACCEL,:));
xtitle('ACCEL');

subplot(nr,nc,2);
plot2d(time, Xsensors(SENSORS_BARO,:));
xtitle('BARO');

endfunction

