clear();
getf('mb_utils.sci');

rank=[];
time_s=[];
time_r=[];
throttle_raw=[];
tacho_raw=[];

u=mopen('data/xxx_prbs.txt','r');
while meof(u) == 0,
  line = mgetl(u, 1);
  if strindex(line, '#') ~= 1 & length(line) ~= 0,  
    [nb_scan, _rank, _time_s, _time_r, _throttle_raw, _tacho_raw] = msscanf(1, line, '%x %x %x %x %x');
    if nb_scan == 5,
      rank = [rank _rank];
      time_s = [time_s _time_s];
      time_r = [time_r _time_r];
      throttle_raw = [throttle_raw _throttle_raw];
      tacho_raw = [tacho_raw _tacho_raw];
    end
  end
end

mclose(u);


// extract experiment
//EXPE_LEN = 4096;
EXPE_LEN = 100;
EXPE_START = 41;
rank = rank(EXPE_START:EXPE_START+EXPE_LEN);
time_s = time_s(EXPE_START:EXPE_START+EXPE_LEN);
time_r = time_r(EXPE_START:EXPE_START+EXPE_LEN);
throttle_raw = throttle_raw(EXPE_START:EXPE_START+EXPE_LEN);
tacho_raw = tacho_raw(EXPE_START:EXPE_START+EXPE_LEN);

// compute time
TICK_PER_S = 15625;
time = time_s + time_r / TICK_PER_S;
DT = (time(length(time)) - time(1)) / (length(time) - 1)
F_EC = 1 / DT

// normalise throttle
MAX_PPRZ = 9600;
throttle = throttle_raw / MAX_PPRZ;

// compute rpms
TICK_PER_ROTATION = 36;
rpm = tacho_raw / DT / TICK_PER_ROTATION * 60;

// integrate ODE
//kv0 = 100000;
kv0 = 22000;
//taukq =  0.0003313;     //
taukq =  0.00001;         //
//tau0 = 0.3;               //
tau0 = 0.1;               //
kq0 = taukq / tau0;       //

rpm0 = rpm(1);

//idx = 1;
function rpm_dot = prbs_mot_ode(t, rpm)
  idx=1;
  while (time(idx) < t & idx < length(throttle)), idx=idx+1, end
//  printf('%f %d %f\n', t, idx, time(idx));
  rpm_dot = mot_ode(my_tau, my_kq, my_kv, throttle(idx), rpm);
endfunction

p0 = [tau0; kv0; kq0];


function e = err_prbs(p,z)
  my_tau = p(1);
  my_kv  = p(2);
  my_kq  = p(3);
  rpm_sim = ode([rpm0], time(1), time, prbs_mot_ode);
  _diff = rpm_sim - rpm;
  sq_diff = _diff^2;
  e = sum(sq_diff);
endfunction


if 0
  my_tau = tau0;
  my_kv = kv0;
  my_kq = kq0;
  rpm_sim = ode([rpm0], time(1), time, prbs_mot_ode);
else
  Z = [ 1; 1]; //unused...
  [p, err] = datafit(err_prbs, Z, p0) 
  my_tau = p(1);
  my_kv = p(2);
  my_kq = p(3); 
  rpm_sim = ode([rpm0], time(1), time, prbs_mot_ode);  
end



xbasc();
subplot(2,1,1);
xtitle('Throttle');
plot(time, throttle, 'b-');

subplot(2,1,2);
plot(time, rpm, 'r-');
plot(time, rpm_sim, 'b-');