clear();
getf('mb_utils.sci');

filename = "data/steps_stout_aero_geared.txt";

[time, throttle, rpm, amp, thrust, torque] = read_mb_log(filename);


// design an iir low pass with cutoff freq at 125Hz
hz=iir(3,'lp','ellip',[0.5 0.5],[.01 .01]);

//[hzm,fr]=frmag(hz,256);
//plot2d(fr',hzm',3)

[rpm_lp] = rtitr(hz(2), hz(3), rpm);


//if 0
  xbasc();
  subplot(2,1,1)
  xtitle('Throttle');
  plot(time(30:length(time)), throttle(30:length(throttle)), 'b-');
  subplot(2,1,2)
  plot(time(30:length(time)), rpm(30:length(rpm)), 'r-');
  plot(time(30:length(time)), rpm_lp(30:length(rpm_lp)), 'g-');
//end

s_avg = 30;
e_avg = 118;
rpm0 = mean(rpm(s_avg:e_avg));

plot(time(s_avg:e_avg), rpm0*ones(1,e_avg - s_avg + 1), 'b-');

my_kv = 100000;
my_taukq = - 0.0003313;     // plain wrong it seems :(
my_tau = 0.3;              //
my_kq = my_taukq / my_tau;  //


time_ode = time(s_avg:length(time));
u_ode = throttle(s_avg:length(throttle));


function rpm_dot = my_mot_ode(t, rpm)
    if ((t >= 389.7960)& (t < 390.7960))
      thr = 0.7;
    else
      thr = 0.6;
    end
    rpm_dot = mot_ode(my_tau, my_kq, my_kv, thr);
 endfunction
 

rpm_sim = ode([rpm0], time(1), time_ode, my_mot_ode);
 
plot(time_ode, rpm_sim, 'r-');