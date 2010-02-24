

function [fo_traj] = fo_traj_circle(time)

  fo_traj = zeros(DF_FO_SIZE, DF_FO_ORDER, length(time));

  radius = 3;
  omega = rad_of_deg(60);
  c = [0 0]';
  
  for i=1:length(time)

    fo_traj(DF_FO_X,1,i) = c(1) + radius*cos(omega*time(i));
    fo_traj(DF_FO_Y,1,i) = c(2) + radius*sin(omega*time(i));

    fo_traj(DF_FO_X,2,i) = c(1) - omega*radius*sin(omega*time(i));
    fo_traj(DF_FO_Y,2,i) = c(2) + omega*radius*cos(omega*time(i));
  
    fo_traj(DF_FO_X,3,i) = c(1) - omega^2*radius*cos(omega*time(i));
    fo_traj(DF_FO_Y,3,i) = c(2) - omega^2*radius*sin(omega*time(i));
    
    fo_traj(DF_FO_X,4,i) = c(1) + omega^3*radius*sin(omega*time(i));
    fo_traj(DF_FO_Y,4,i) = c(2) - omega^3*radius*cos(omega*time(i));
    
    fo_traj(DF_FO_X,5,i) = c(1) + omega^4*radius*cos(omega*time(i));
    fo_traj(DF_FO_Y,5,i) = c(2) + omega^4*radius*sin(omega*time(i));
  
  end
    
endfunction

