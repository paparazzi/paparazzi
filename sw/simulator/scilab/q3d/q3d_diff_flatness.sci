//
// Differential Flatness
//


DF_OX    = 1;  // Flat output first  coordinate
DF_OZ    = 2;  // Flat output second coordinate
DF_OSIZE = 2;
DF_ORANK = 5;  // Number of time derivative needed

fo_g       = 9.81;
fo_mass    = 0.25;
fo_inertia = 0.0078;

global fo_traj;

// state from flat output
function [state] = df_state_of_fo(fo)
  state = zeros(FDM_SSIZE, 1);
  state(FDM_SX)      = fo(1,1);
  state(FDM_SZ)      = fo(2,1);
  state(FDM_SXD)     = fo(1,2);
  state(FDM_SZD)     = fo(2,2);
  theta = -atan(fo(1,3), fo_g + fo(2,3));
  state(FDM_STHETA)  = theta;
  thetad = -((fo_g + fo(2,3))*fo(1,4) - fo(1,3)*fo(2,4)) / ...
      ((fo_g + fo(2,3))^2+fo(1,3)^2);
  state(FDM_STHETAD) = thetad;
endfunction

// control input from flat output
function [inp] = df_input_of_fo(fo)

  x2   = fo(1,3);				 
  z2p1 = fo(2,3)+9.81;				 

  u1 =  fo_mass * sqrt((x2)^2 + (z2p1)^2);
  
  x3   = fo(1,4);				 
  z3   = fo(2,4);				 
  x4   = fo(1,5);				 
  z4   = fo(2,5);				 
  a = x4*z2p1 - z4*x2; 				 
  b = z2p1^2+x2^2;
  c = 2 * (z2p1*z3 + x2*x3);
  d = x3*z2p1-z3*x2;
  u2 = -fo_inertia * ( a/b - c*d/b^2);
 
  inp = [u1; u2];
  
endfunction


// 
function [fo] = df_get_traj_polynomial(Xin, Xout, Uin, Uout, time)

  fo = zeros(DF_OSIZE*DF_ORANK, length(time));
  // boundary conditions
  fo(1,1)  = Xin(FDM_SX);
  fo(2,1)  = Xin(FDM_SZ);
  fo(3,1)  = Xin(FDM_SXD);
  fo(4,1)  = Xin(FDM_SZD);
  fo(5,1)  = -Uin(1)*sin(Xin(FDM_STHETA));
  fo(6,1)  =  Uin(1)*(cos(Xin(FDM_STHETA)) - 1);
  fo(7,1)  = 0;
  fo(8,1)  = 0;
  fo(9,1)  = 0;
  fo(10,1) = 0;
  
  fo(1, length(time)) = Xout(FDM_SX);
  fo(2, length(time)) = Xout(FDM_SZ);
  fo(3, length(time)) = Xout(FDM_SXD);
  fo(4, length(time)) = Xout(FDM_SZD);
  fo(5, length(time)) = -Uout(1)*sin(Xout(FDM_STHETA));
  fo(6, length(time)) =  Uout(1)*(cos(Xout(FDM_STHETA)) - 1);
  fo(7, length(time)) = 0;
  fo(8, length(time)) = 0;
  fo(9, length(time)) = 0;
  fo(10,length(time)) = 0;  

 
endfunction





