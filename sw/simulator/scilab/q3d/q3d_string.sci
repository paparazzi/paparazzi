


function [traj] = string_get_traj()

  M=fscanfMat('fonts/e_cms.csv');

  X = M(:,1);
  Y = M(:,2);
  on= M(:,3);

  X = [X; X(1)];
  Y = [Y; Y(1)];

  delta_x = max(X) - min(X);
  sf = 3/delta_x;
  X = sf * X;
  Y = sf * Y;

  speed = 2;
  dt = 1/512;

  a0 = [X(1); Y(1)];
  traj = [[0; 0]];
  i = 1;
  while i < length(X)
    a1 = [X(i); Y(i)];
    a2 = [X(i+1); Y(i+1)];
    v = a2 - a1;
    np = norm(v) / speed / dt;
    for j=1:np
      traj = [traj a2-a0];
    end
    i = i+1;
  end

  traj = [traj zeros(2,1024)];


endfunction

function [traj1, traj5] = string_get_traj2()

  M=fscanfMat('fonts/e_cms.csv');

  X = M(:,1);
  Y = M(:,2);
  on= M(:,3);

  // scale to 3m lateral amplitude
  delta_x = max(X) - min(X);
  sf = 3/delta_x;
  X = sf * X;
  Y = sf * Y;
  // offset to start at [0;0]
  X = X - X(1);
  Y = Y - Y(1);

  traj1 = [X';Y'];


  X = [X; X(1)];
  Y = [Y; Y(1)];
  traj5 = [];

//  for i=1:length(X)
    i = 1;
    p0 = [X(1); Y(1)];
    p1 = [X(2); Y(2)];
    p2 = [X(3); Y(3)];
    p3 = [X(4); Y(4)];
    for t=0:dt:1
      p = (1-t)^3*p0 + 3*(1-t)^2*t*p1 + 3*(1-t)*t^2*p2 + t^3*p3;
      traj5 = [traj5 p];
    end

    //  traj5 = [X';Y'];
//  end

endfunction

