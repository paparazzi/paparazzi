
%
% build synthetic imu data
%
function [gyro, accel, mag] =  synth_imu(rates, quat)

nb_samples = length(rates);

g_ned = [ 0 
	  0 
	  258.3275];

h_ned = [ 166.8120 
	  0.0 
	  203.3070];

for idx = 1:nb_samples

  dcm = dcm_of_quat(quat(:, idx));
  accel(:, idx) = sim_accel(g_ned, dcm);
  mag(:, idx) = sim_mag(h_ned, dcm);
  gyro(:, idx) = sim_gyro_2(rates(:, idx));
  
end;
