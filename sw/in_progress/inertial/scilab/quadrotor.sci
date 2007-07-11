//
//
//
// trajectory generation
//
//
//

//
// phi_dot   = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r
// theta_dot =     cos(phi)           *q - sin(phi)           *r
// psi_dot   =     sin(phi)/cos(theta)*q + cos(phi)/cos(theta)*r
//

function [time, rates, eulers] = quadrotor_gen_roll_step(euler0, dt)
  len_gen = 20.;
  t0 =  5.;
  t1 =  7.;
  t2 = 13.;
  t3 = 15.;
  ampl = 0.4;
  nb_samples = len_gen / dt;
  time=[0.];
  rates=[[0.; 0.; 0.]];
  eulers=[euler0];
  for i=2:nb_samples
    t = (i-1)*dt;
    time = [time t];
    rate = [0.; 0.; 0.];
    if (t >= t0 & t <= t1),
      rate = rate + [ampl * sin(%pi/(t1-t0)*(t-t0)); 0.; 0.];
    elseif (t >= t2 & t <= t3), 
      rate = rate + [-ampl * sin(%pi/(t1-t0)*(t-t0)); 0.; 0.];
    end
    rates  = [rates rate];
    phi   = eulers(1,i-1);
    theta = eulers(2,i-1);
    psi   = eulers(3,i-1);
    p = rates(1, i-1);
    q = rates(2, i-1);
    r = rates(3, i-1);
    euler_dot = [ p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r
	              cos(phi)           *q - sin(phi)           *r
		      sin(phi)/cos(theta)*q + cos(phi)/cos(theta)*r ];
    euler = eulers(:,i-1) + euler_dot * dt;
    eulers = [eulers euler];
  end

endfunction

function [time, rates, eulers] = quadrotor_gen_cste_rot(euler0, dt)
  len_gen = 20.;
  nb_samples = len_gen / dt;
  omega_1 = rad_of_deg(150);
  omega_2 = rad_of_deg(120);
    
  eulers=[euler0];
  for i=2:nb_samples
    t = (i-1)*dt;
    time = [time t];
    euler = [ t * omega_1
	      0
	      0 ] + euler0;
    
    
  end
endfunction

