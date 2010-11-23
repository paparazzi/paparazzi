

function [fo_traj] = fo_traj_circle(time, _center, radius, omega)

  n_comp = 2;
  order = 5;
  fo_traj = zeros(n_comp, order, length(time));

  for i=1:length(time)

    alpha = omega*time(i);
    fo_traj(1,1,i) = _center(1) + radius * cos(alpha);
    fo_traj(2,1,i) = _center(1) + radius * sin(alpha);

    fo_traj(1,2,i) = -omega * radius * sin(alpha);
    fo_traj(2,2,i) =  omega * radius * cos(alpha);

    fo_traj(1,3,i) = -omega^2 * radius * cos(alpha);
    fo_traj(2,3,i) = -omega^2 * radius * sin(alpha);

    fo_traj(1,4,i) =  omega^3 * radius * sin(alpha);
    fo_traj(2,4,i) = -omega^3 * radius * cos(alpha);

    fo_traj(1,5,i) =  omega^4 * radius * cos(alpha);
    fo_traj(2,5,i) =  omega^4 * radius * sin(alpha);

  end


endfunction


function [fo_traj] = fo_traj_swing(time)
  omega = 2.;
  n_comp = 2;
  order = 5;
  fo_traj = zeros(n_comp, order, length(time));

 for i=1:length(time)

    alpha = omega*time(i);
    radius = 2;

    fo_traj(1,1,i) = radius * cos(alpha);

    fo_traj(1,2,i) = -omega * radius * sin(alpha);

    fo_traj(1,3,i) = -omega^2 * radius * cos(alpha);

    fo_traj(1,4,i) =  omega^3 * radius * sin(alpha);

    fo_traj(1,5,i) =  omega^4 * radius * cos(alpha);

  end




endfunction





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
