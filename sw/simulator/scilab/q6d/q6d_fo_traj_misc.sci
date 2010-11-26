function [time_out, traj_out] = merge_traj(time_in, traj_in)
  time_out = [];
  for t=time_in
    time_out = [time_out t];
  end
  traj_out = 0;
  [nb_comp, nb_order, foo] = size(traj_in(1));
  traj_out = zeros(nb_comp, nb_order, length(time_out));

  l=1;
  for i=1:length(time_in)
    for j=1:length(time_in(i))
      ti = traj_in(i);
      traj_out(:,:,l) = ti(:,:,j);
      l=l+1;
    end
  end
endfunction


function [fo_traj] = fo_traj_circle(time)

  fo_traj = zeros(DF_FO_SIZE, DF_FO_ORDER, length(time));

  radius = 3;
  omega = rad_of_deg(60);
  c = [0 0]';
  omega_z = rad_of_deg(105);
  dz = 1.;

  for i=1:length(time)

    fo_traj(DF_FO_X,1,i) = c(1) + radius*cos(omega*time(i));
    fo_traj(DF_FO_Y,1,i) = c(2) + radius*sin(omega*time(i));
    fo_traj(DF_FO_Z,1,i) = dz*sin(omega_z*time(i));
//    fo_traj(DF_FO_PSI,1,i) = omega*time(i);


    fo_traj(DF_FO_X,2,i) = c(1) - omega*radius*sin(omega*time(i));
    fo_traj(DF_FO_Y,2,i) = c(2) + omega*radius*cos(omega*time(i));
    fo_traj(DF_FO_Z,2,i) = omega_z*dz*cos(omega_z*time(i));
//    fo_traj(DF_FO_PSI,2,i) = omega;


    fo_traj(DF_FO_X,3,i) = c(1) - omega^2*radius*cos(omega*time(i));
    fo_traj(DF_FO_Y,3,i) = c(2) - omega^2*radius*sin(omega*time(i));
    fo_traj(DF_FO_Z,3,i) = -omega_z^2*dz*sin(omega_z*time(i));
//    fo_traj(DF_FO_PSI,3,i) = 0;


    fo_traj(DF_FO_X,4,i) = c(1) + omega^3*radius*sin(omega*time(i));
    fo_traj(DF_FO_Y,4,i) = c(2) - omega^3*radius*cos(omega*time(i));
    fo_traj(DF_FO_Z,4,i) = -omega_z^3*dz*cos(omega_z*time(i));


    fo_traj(DF_FO_X,5,i) = c(1) + omega^4*radius*cos(omega*time(i));
    fo_traj(DF_FO_Y,5,i) = c(2) + omega^4*radius*sin(omega*time(i));
    fo_traj(DF_FO_Z,5,i) = omega_z^4*dz*sin(omega_z*time(i));

  end

endfunction

