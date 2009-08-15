//
// compute neutral and scale factor from a serie of idealy homogeneously spread raw
// measurements components, assuming the norme of the measured quantity is constant
//

clear();

exec("calibrate_utils.sci");

ac_id = 15;
//log_name = '/home/john.stower/paparazzi3/var/logs/08_11_24__12_53_41.data';
//log_name = 'log_x1_mag_raw';
log_name = 'log_accel_mercury3';
//log_name = 'log_accel_booz2_a1_5';



sensor_type = SENSOR_ACCEL;

select sensor_type

case SENSOR_ACCEL then 
  sensor_name = "ACCEL";
  ref_norm = 9.81;
  resolution = 2^10;
  reject_noisy_data = 1;
  threshold = 4000;
  size_avg = 10;

case SENSOR_MAG then
  sensor_name = "MAG";
  ref_norm = 1.;
  resolution = 2^11;
  reject_noisy_data = 0;

end


// read log
[time, sensor_raw] = read_log_sensor_raw(ac_id, sensor_type, log_name);

if 0
pause
for i=1:3
  for j=1:length(time)
    if sensor_raw(i,j) < 0
      sensor_raw(i,j) = sensor_raw(i,j) + 2^16;
    end
  end
end
end

// plot raw sensors
scf();
display_raw_sensors(time, sensor_raw);

// reject noisy data
if reject_noisy_data
  [time_filtered, sensor_filtered] = filter_noisy_data(time, sensor_raw, threshold, size_avg);
  time = time_filtered;
  sensor_raw = sensor_filtered;
end

// compute intial calibration
[p0] = min_max_calib(sensor_raw, ref_norm);

// plot initial guess calibrated sensors
[mic, gmic] = apply_scaling( p0(1:3), p0(4:6), sensor_raw); 
scf();
display_calib_sensor(time, mic, gmic, 'initial calibration', ref_norm);

// optimise initial calibration
[p, err] = datafit_calib(sensor_raw, ref_norm, p0);
//p = p0;
printf("datafit error : %f\n", err);

// plot optimized calibrated sensors
[mfc, gmfc] = apply_scaling( p(1:3), p(4:6), sensor_raw); 
scf();
display_calib_sensor(time, mfc, gmfc, 'final calibration', ref_norm);

// print xml for airframe file
print_xml(sensor_name, p, resolution);