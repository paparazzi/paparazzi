//
//
// Smoothed bang bang trajectory
//
//

SBB_TOLERANCE = 0.025;

function [step_dt, step_ampl, traj_dt] = compute_step(dist, dyn, max_accel, max_speed)
  if (dist < 0.01)
    step_dt = 0;
    step_ampl = 0;
  else
    omega = dyn(1);
    xsi = dyn(2);
    step_dt = -log(SBB_TOLERANCE)/(omega*xsi*sqrt(1-xsi^2));
    step_ampl = dist / (2*step_dt^2);
    if step_ampl < max_accel
      if step_ampl > max_speed/step_dt
	step_ampl = max_speed/step_dt;
      end
    else
      step_ampl = max_accel;
      if step_ampl <= max_speed/step_dt
	step_dt = max_speed/step_ampl;
      else
	step_ampl = max_speed/step_dt;
      end
    end
    traj_dt = (dist - 2*step_dt*(step_ampl*step_dt))/step_ampl*step_dt + 4*step_dt;
  end
endfunction



function [fo_traj] = sbb_gen_traj(time, dyn, max_speed, max_accel, b0, b1)

  n_comp = 4;  // x, y, z, psi
  order = 5;   
  fo_traj = zeros(n_comp, order, length(time));
  
  disp_xy = b1(1:2) - b0(1:2);
  [pulse_dt, pulse_ampl, traj_dt] = compute_step(norm(disp_xy), dyn(1,:), max_accel(1), max_speed(1));
  fo_traj(1:2,1,1) = b0(1:2);
  u = disp_xy' / norm(disp_xy);
  for i=2:length(time)
    delta_t = time(i)-time(1);
    if delta_t < pulse_dt
      sp = pulse_ampl;
    elseif  delta_t >= traj_dt -  2 * pulse_dt & delta_t < traj_dt - pulse_dt
      sp = -pulse_ampl;
    else
      sp = 0;
    end
    fo_traj(1:2,1:4,i) = fo_traj(1:2,1:4,i-1) + fo_traj(1:2,2:5,i-1)* (time(i)-time(i-1));
    fo_traj(1:2,5,i) = -2*dyn(1,2)*dyn(1,1)*fo_traj(1:2,4,i)-dyn(1,1)^2*(fo_traj(1:2,3,i)-sp*u);  
  end
  
  disp_z = b1(3) - b0(3);
  [pulse_dt, pulse_ampl, traj_dt] = compute_step(norm(disp_z), dyn(2,:), max_accel(2), max_speed(2));
  fo_traj(3,1,1) = b0(3);
  for i=2:length(time)
    delta_t = time(i)-time(1);
    if delta_t < pulse_dt
      sp = pulse_ampl;
    elseif  delta_t >= traj_dt -  2 * pulse_dt & delta_t < traj_dt - pulse_dt
      sp = -pulse_ampl;
    else
      sp = 0;
    end
    fo_traj(3,1:4,i) = fo_traj(3,1:4,i-1) + fo_traj(3,2:5,i-1)*(time(i)-time(i-1));
    fo_traj(3,5,i) = -2*dyn(2,2)*dyn(2,1)*fo_traj(3,4,i)-dyn(2,1)^2*(fo_traj(3,3,i)-sp*sign(disp_z));
  end
  
endfunction




