


function io_dump_fdm_sensor_dat(time, filename)

  fid = mopen(filename+'.txt', "w");
  for i=1:length(time)
    mfprintf(fid, "%f [%.16f %.16f %.16f %.16f] [%.16f %.16f %.16f] [%.16f %.16f %.16f] [%.16f %.16f %.16f] [%.16f %.16f %.16f]\n", ...
	     time(i),...
	     fdm_state(FDM_SQI,i), fdm_state(FDM_SQX,i), fdm_state(FDM_SQY,i), fdm_state(FDM_SQZ,i),...
	     fdm_state(FDM_SP,i), fdm_state(FDM_SQ,i), fdm_state(FDM_SR,i),...
	     sensor_gyro(1,i), sensor_gyro(2,i), sensor_gyro(3,i),...
	     sensor_accel(1,i), sensor_accel(2,i), sensor_accel(3,i),...
	     sensor_mag(1,i), sensor_mag(2,i), sensor_mag(3,i) );
  end
  mclose(fid);
  save(filename+'.dat',time, fdm_state, sensor_gyro, sensor_accel, sensor_mag);

endfunction

