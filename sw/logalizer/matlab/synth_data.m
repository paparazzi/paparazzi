
%
% build synthetic data
%
function [t, rates, quat] = synth_data(dt, nb_samples)

t_end = dt * (nb_samples - 1);
t = 0:dt:t_end;

rates = zeros(3, nb_samples);

omega_q = 15;
amp_q = 2;

osc_start = floor(nb_samples/2);
osc_end = floor(osc_start+2*pi/(omega_q*dt));
for idx=osc_start:osc_end
  rates(2, idx) = -amp_q*(1 - cos((idx-osc_start)*(omega_q*dt)));
end

osc_start = floor(nb_samples/2 + 8*pi/(omega_q*dt));
osc_end = floor(osc_start+2*pi/(omega_q*dt));
for idx=osc_start:osc_end
  rates(2, idx) = amp_q*(1 - cos((idx-osc_start)*(omega_q*dt)));
end

quat(:, 1) = quat_of_eulers([0.2 -0.4 0.5]');

for idx=2:nb_samples
  p = rates(1, idx-1);
  q = rates(2, idx-1);
  r = rates(3, idx-1);
  
  omega = [ 0 -p -q -r
	    p  0  r -q
	    q -r  0  p
	    r  q -p  0 ];
  
  quat_dot = 0.5 * omega * quat(:, idx-1);
  quat(:, idx) = quat(:, idx-1) + quat_dot * dt;
  quat(:, idx) = normalize_quat(quat(:, idx));

end


