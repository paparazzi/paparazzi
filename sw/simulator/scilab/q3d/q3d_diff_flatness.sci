//
// Differential Flatness
//


DF_OX    = 1;  // Flat output first  coordinate
DF_OZ    = 2;  // Flat output second coordinate
DF_OSIZE = 2;
DF_ORANK = 5;  // Number of time derivative needed

fo_g       = 9.81;
fo_mass    = 0.25;
fo_J       = 0.0078;
fo_Ct0     = 4. * fo_mass * fo_g / 2;
fo_la      = 0.25;

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
function [inp] = df_input_of_fo(fo, model_a, model_b)

  x2   = fo(1,3);
  z2pg = fo(2,3)+9.81;

//  u1 =  fo_mass / fo_Ct0 * sqrt((x2)^2 + (z2pg)^2);
  u1 =  1/model_a * sqrt((x2)^2 + (z2pg)^2);

  x3   = fo(1,4);
  z3   = fo(2,4);
  x4   = fo(1,5);
  z4   = fo(2,5);
  a = x4*z2pg - z4*x2;
  b = z2pg^2+x2^2;
  c = 2 * (z2pg*z3 + x2*x3);
  d = x3*z2pg-z3*x2;
//  u2 = -fo_J / fo_la /fo_Ct0 * ( a/b - c*d/b^2);
  u2 = -1/model_b * ( a/b - c*d/b^2);

  inp = [u1; u2];

endfunction





