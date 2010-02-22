//
//
// Smoothed bang bang trajectory
//
//

sbb_omega = rad_of_deg(500);
sbb_ksi = 0.70;
sbb_tolerance = 0.05;

function [fo_traj] = sbb_gen_traj(time, max_xd, max_theta, b0, b1)
  n_comp = 2;
  order = 5;
  fo_traj = zeros(n_comp, order, length(time));

  // compute trajectory caracteristics
  dx = b1(1)-b0(1);
  step_dt = -log(sbb_tolerance)/(sbb_ksi*sbb_omega*sqrt(1-sbb_ksi^2));
  step_xdd = dx / (2*step_dt^2);
  max_xdd = 9.81 * tan(max_theta);
  
  if step_xdd < max_xdd
    if step_xdd > max_xd/step_dt
      step_xdd = max_xd/step_dt;
    end
  else
    step_xdd = max_xdd;
    if step_xdd <= max_xd/step_dt
      step_dt = max_xd/step_xdd;
    else
      step_xdd = max_xd/step_dt;
    end
  end

  t_tot = (dx - 2*step_dt*(step_xdd*step_dt))/(step_xdd*step_dt) + 4*step_dt;

  printf('dx        :%f\n', dx);
  printf('step_dt   :%f\n', step_dt);
  printf('step_xdd  :%f\n', step_xdd);
  printf('total time:%f\n', t_tot);
  
  
  fo_traj(1,1,1) = b0(1);
  for i=2:length(time)
    if time(i) < step_dt
      sp = step_xdd;      
    elseif time(i) < t_tot - step_dt & time(i) >= t_tot -  2 * step_dt
      sp = -step_xdd;      
    else
      sp = 0;
    end
    fo_traj(1,:,i) = propagate_traj(fo_traj(1,:,i-1), sp, time(i) - time(i-1)); 
  end

endfunction


function [Xi1] = propagate_traj(Xi, sp, dt)
  Xi1 = zeros(1,5);
  Xi1(1:4) = Xi(1:4) +  Xi(2:5)*dt;
  Xi1(5) = -2*sbb_ksi*sbb_omega*Xi1(4)-sbb_omega^2*(Xi1(3)-sp);
endfunction
