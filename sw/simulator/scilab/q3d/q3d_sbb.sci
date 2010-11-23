//
//
// Smoothed bang bang trajectory
//
//

sbb_tolerance = 0.025;

function [fo_traj] = sbb_gen_traj(time, dyn, max_speed, max_accel, b0, b1)
  n_comp = 2;
  order = 5;
  fo_traj = zeros(n_comp, order, length(time));

  for compo=1:n_comp
    // compute trajectory caracteristics
    dist = b1(compo)-b0(compo);
    if (abs(dist) > 0.01)
      step_dt = -log(sbb_tolerance)/(dyn(compo,2)*dyn(compo,1)*sqrt(1-dyn(compo,2)^2));
      step_xdd = abs(dist) / (2*step_dt^2);

      if step_xdd < max_accel(compo)
        if step_xdd > max_speed(compo)/step_dt
          step_xdd = max_speed(compo)/step_dt;
        end
      else
        step_xdd = max_accel(compo);
        if step_xdd <= max_speed(compo)/step_dt
          step_dt = max_speed(compo)/step_xdd;
        else
          step_xdd = max_speed(compo)/step_dt;
        end
      end
      t_tot = (abs(dist) - 2*step_dt*(step_xdd*step_dt))/(step_xdd*step_dt) + 4*step_dt;
    else
      step_dt = 0;
      step_xdd = 0;
      t_tot = 0;
    end
    printf('dist      :%f\n', dist);
    printf('step_dt   :%f\n', step_dt);
    printf('step_xdd  :%f\n', step_xdd);
    printf('total time:%f\n', t_tot);


    fo_traj(compo,1,1) = b0(compo);
      for i=2:length(time)
        if time(i)-time(1) < step_dt
          sp = sign(dist)*step_xdd;
        elseif time(i)-time(1) < t_tot - step_dt & time(i)-time(1) >= t_tot -  2 * step_dt
          sp = -sign(dist)*step_xdd;
        else
          sp = 0;
        end
        fo_traj(compo,:,i) = propagate_traj(fo_traj(compo,:,i-1), dyn(compo,:), sp, time(i) - time(i-1));
      end
    end

endfunction

function [Xi1] = propagate_traj(Xi, dyn, sp, dt)
  Xi1 = zeros(1,5);
  Xi1(1:4) = Xi(1:4) +  Xi(2:5)*dt;
  Xi1(5) = -2*dyn(2)*dyn(1)*Xi1(4)-dyn(1)^2*(Xi1(3)-sp);
endfunction
