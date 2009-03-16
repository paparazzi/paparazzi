


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

