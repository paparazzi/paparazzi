//
//
// Smoothed bang bang trajectory
//
//



SBB_TOLERANCE = 0.025;

function [pulse_dt, pulse_ampl, traj_dt] = compute_step(dist, dyn, max_accel, max_speed)
  if (dist < 0.01)
    pulse_dt = 0;
    pulse_ampl = 0;
    traj_dt = 1;
  else
    omega = dyn(1);
    xsi = dyn(2);
    pulse_dt = -log(SBB_TOLERANCE)/(omega*xsi*sqrt(1-xsi^2));
    pulse_ampl = dist / (2*pulse_dt^2);
    if pulse_ampl < max_accel
      if pulse_ampl > max_speed/pulse_dt
	pulse_ampl = max_speed/pulse_dt;
      end
    else
      pulse_ampl = max_accel;
      if pulse_ampl <= max_speed/pulse_dt
	pulse_dt = max_speed/pulse_ampl;
      else
	pulse_ampl = max_speed/pulse_dt;
      end
    end
    traj_dt = (dist - 2*pulse_dt*(pulse_ampl*pulse_dt))/pulse_ampl*pulse_dt + 4*pulse_dt;
  end
endfunction



function [fo_traj] = sbb_gen_traj(time, dyn, max_speed, max_accel, b0, b1)

  fo_traj = zeros(DF_FO_SIZE, DF_FO_ORDER, length(time));

  // psi
if 1
 omega = rad_of_deg(45);
  for i=1:length(time)
    fo_traj(DF_FO_PSI, 1, i) =          sin(omega*time(i));
    fo_traj(DF_FO_PSI, 2, i) =  omega*  cos(omega*time(i));
    fo_traj(DF_FO_PSI, 3, i) = -omega^2*sin(omega*time(i));
  end
end
  // x and y
  disp_xy = b1(1:2) - b0(1:2);
  [pulse_dt, pulse_ampl, traj_dt] = compute_step(norm(disp_xy), dyn(1,:), max_accel(1), max_speed(1));
  fo_traj(1:2,1,1) = b0(1:2);
  if norm(disp_xy) != 0
    u = disp_xy' / norm(disp_xy);
  else
    u = [0 0]';
  end
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

  // z
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




