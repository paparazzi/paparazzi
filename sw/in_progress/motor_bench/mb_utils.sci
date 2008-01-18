

function [time, throttle, rpm, amp, thrust, torque] = read_mb_log(filename)
  
  time=[];
  throttle=[];
  rpm=[];
  amp=[];
  thrust=[];
  torque=[];
  
  u=mopen(filename,'r');
  
  while meof(u) == 0,
    line = mgetl(u, 1);
    if strindex(line, '#') ~= 1 & length(line) ~= 0,
      [nb_scan, _time, _throttle, _rpm, _amp, _thrust, _torque] = msscanf(1, line, '%f %f %f %f %f %f');
      if nb_scan == 6,
	time = [time _time];
	throttle = [throttle _throttle];
	rpm = [rpm _rpm];
	amp = [amp _amp];
	thrust = [thrust _thrust];
	torque = [torque _torque];
      end
    end
  end
  
  mclose(u);
  
endfunction

function [averaged] = average(nb_points, raw_data_throttle, raw_data_rpm)
  av_rpm=zeros(1,nb_points+1);
  count=zeros(1,nb_points+1);
  //Summation of the Data
  for i=1:length(raw_data_throttle)
    idx=round(raw_data_throttle(i)*nb_points)+1;
    av_rpm(idx) = av_rpm(idx) +  raw_data_rpm(i);
    count(idx) = count(idx) + 1;
  end
  
  // Dividing the Data by the summated points
  
  for i=1:nb_points+1          
    if count(i) ~=0
      av_rpm(i) = av_rpm(i) / count(i);
      // printf('i=%d count=%d av_rpm=%d\n', i, count(i), av_rpm(i) );
    end
  end
  
  averaged = av_rpm;
  
endfunction


function [filtered] = low_pass_filter(f_sample, f_cut, raw_data)
  
  delta_t = 1/f_sample;
  rc = 1 / ( 2 * %pi * f_cut);
  alpha = delta_t / ( delta_t + rc );
  
  filtered=[raw_data(1)];
  for i=2:length(raw_data)
    fv = alpha * raw_data(i) + (1 - alpha) * filtered(i-1);
    filtered = [filtered fv];
  end
  
endfunction



// Definition of the Fittin Function 
// FF=Fitting Function 
// x=Vector of Variables
// p=Vector of Parameters
function y=FF(x,p)
  y=p(1)+((sqrt(p(2)*x) +1)-1)/p(3)
endfunction

//The criterion function
//Discrete errorfunction
//z = x Vector and y Vector of the collected Data
function e=G(p,z),
  y=z(1),x=z(2);
  e=y-FF(x,p),
endfunction

function [] = param_fit(av_rpm, av_throttle)
  
  X=av_throttle;
  Y=av_rpm;
  Z=[X;Y];
  //solve the Problem
  //Initial parameters
  p0 = [1;1;1];
  //call the datafit function with following parameter:
  //G = errorfunction
  //Z = collected Data Vector
  //p0= initial parameter
  [p,err] = datafit(G,Z,p0);
  
  printf('\n')
  printf('p=%f\n', p)
  
  scf(0);clf()
  //plot2d(X,FF(X,pg),5) //the curve without noise
  subplot(2,1,1);
  plot2d(X,Y,1)  // the noisy data
   subplot(2,1,2);
  plot2d(X,FF(X,p),12) //the solution
  param_fitted=FF(X,p);
endfunction


function r = mot_lin(t, p)
   r = p(1) * t + p(2);
 endfunction
 
function e = err_mot_lin(p, z)
   r=z(1), t=z(2); 
   e = r - mot_lin(t, p);
 endfunction
 
 
function rpm = mot_ric_stat(throttle,param)
 // maybe 1 +
 tau_kq = param(1);
 kv = param(2);
 dv = 0.0;
 rpm = (-1 + sqrt(1 + 4 * tau_kq * kv * (throttle + dv))) / (2 * tau_kq);
endfunction

function e = err_mot_ric(p, z)
  r_meas = z(1);
  throttle = z(2);
  e = r_meas - mot_ric_stat(throttle, p);
endfunction



function rpm_dot = mot_ode(tau, kq, kv, V, rpm)
  rpm_dot = -1/tau*rpm - kq * rpm^2 + kv/tau*(V);
endfunction