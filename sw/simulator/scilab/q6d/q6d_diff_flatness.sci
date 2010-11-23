DF_FO_X   = 1;
DF_FO_Y   = 2;
DF_FO_Z   = 3;
DF_FO_PSI = 4;
DF_FO_SIZE = 4;
DF_FO_ORDER = 5;

DF_REF_X      = 1;
DF_REF_Y      = 2;
DF_REF_Z      = 3;
DF_REF_PHI    = 4;
DF_REF_THETA  = 5;
DF_REF_PSI    = 6;
DF_REF_XD     = 7;
DF_REF_YD     = 8;
DF_REF_ZD     = 9;
DF_REF_P      = 10;
DF_REF_Q      = 11;
DF_REF_R      = 12;
DF_REF_SIZE   = 12;

DF_G = 9.81;
DF_MASS = 0.5;
DF_JXX = 0.0078;
DF_JYY = 0.0078;
DF_JZZ = 0.0137;
DF_L = 0.25;
DF_CM_OV_CT = 0.1;

// state from flat output
function [ref] = df_ref_of_fo(fo)

  ref = zeros(DF_REF_SIZE, 1);
  ref(DF_REF_X)  = fo(DF_FO_X,1);
  ref(DF_REF_Y)  = fo(DF_FO_Y,1);
  ref(DF_REF_Z)  = fo(DF_FO_Z,1);

  ref(DF_REF_XD) = fo(DF_FO_X,2);
  ref(DF_REF_YD) = fo(DF_FO_Y,2);
  ref(DF_REF_ZD) = fo(DF_FO_Z,2);

  ref(DF_REF_PSI) = fo(DF_FO_PSI,1);

  psi = ref(DF_REF_PSI);
  cpsi = cos(psi);
  spsi = sin(psi);

  x2d = fo(1,3);
  y2d = fo(2,3);
  z2d = fo(3,3);

  axpsi = cpsi*x2d + spsi*y2d;
  aypsi = spsi*x2d - cpsi*y2d;
  z2dmg = z2d - DF_G;
  av = sqrt(axpsi^2 + z2dmg^2);

  ref(DF_REF_PHI) = sign(z2dmg)*atan(aypsi/av);
  ref(DF_REF_THETA) = atan(axpsi/z2dmg);

  x3d = fo(1,4);
  y3d = fo(2,4);
  z3d = fo(3,4);

  jxpsi = cpsi*x3d + spsi*y3d;
  jypsi = spsi*x3d - cpsi*y3d;

  x4d = fo(1,5);
  y4d = fo(2,5);

  kxpsi = cpsi*x4d + spsi*y4d;
  kypsi = spsi*x4d - cpsi*y4d;

  psid = fo(4,2);

  adxpsi = -psid*aypsi + jxpsi;
  adypsi =  psid*axpsi + jypsi;

  adv = (axpsi*adxpsi + z2dmg*fo(3,4))/av;

  phid   = sign(z2dmg)*(adypsi*av-adv*aypsi)/(aypsi^2+av^2);
  thetad = (adxpsi*z2dmg-z3d*axpsi)/(axpsi^2+z2dmg^2);

  cphi = cos(ref(DF_REF_PHI));
  sphi = sin(ref(DF_REF_PHI));
  ctheta = cos(ref(DF_REF_THETA));
  stheta = sin(ref(DF_REF_THETA));

  ref(DF_REF_P) =  phid - stheta*psid;
  ref(DF_REF_Q) =  cphi*thetad + sphi*ctheta*psid;
  ref(DF_REF_R) = -sphi*thetad + cphi*ctheta*psid;

endfunction




// control input from flat output
function [inp] = df_input_of_fo(fo)

  inp = zeros(4,1);

  x2d = fo(1,3);
  y2d = fo(2,3);
  z2d = fo(3,3);
  z2dmg = z2d - DF_G;
  inp(1) = DF_MASS * sqrt(x2d^2+y2d^2+z2dmg^2);

  psi   = fo(4,1);
  psid  = fo(4,2);
  psi2d = fo(4,3);

  cpsi = cos(psi);
  spsi = sin(psi);

  axpsi = cpsi*x2d + spsi*y2d;
  aypsi = spsi*x2d - cpsi*y2d;

  x3d = fo(1,4);
  y3d = fo(2,4);

  jxpsi = cpsi*x3d + spsi*y3d;
  jypsi = spsi*x3d - cpsi*y3d;

  x4d = fo(1,5);
  y4d = fo(2,5);

  kxpsi = cpsi*x4d + spsi*y4d;
  kypsi = spsi*x4d - cpsi*y4d;

  adxpsi = -psid*aypsi + jxpsi;
  adypsi =  psid*axpsi + jypsi;

  a2dxpsi = -psi2d*aypsi - psid^2*axpsi - 2*psid*jypsi + kxpsi;
  a2dypsi =  psi2d*axpsi - psid^2*aypsi + 2*psid*jxpsi + kypsi;

  av = sqrt(axpsi^2 + z2dmg^2);
  z3d = fo(3,4);
  adv = (axpsi*adxpsi + z2dmg*z3d)/av;
  z4d = fo(3,5);
  a = (axpsi*a2dxpsi + adxpsi^2 + (z2dmg)*z4d +z3d^2)*av;
  b = -adv*(axpsi*adxpsi + z2dmg*z3d);
  a2dv = (a+b)/av^2;

  phi = sign(z2dmg)*atan(aypsi/av);
  theta = atan(axpsi/z2dmg);

  phid   = sign(z2dmg)*(adypsi*av-adv*aypsi)/(aypsi^2+av^2);
  thetad = (adxpsi*z2dmg-z3d*axpsi)/(axpsi^2+z2dmg^2);

  a = (a2dypsi*av-a2dv*aypsi)*(aypsi^2+av^2);
  b = -2*(aypsi*adypsi+av*adv)*(adypsi*av-adv*aypsi);
  c = (aypsi^2+av^2)^2;
  phi2d = sign(z2dmg)*(a+b)/c;

  a = (a2dxpsi*z2dmg-z4d*axpsi)*(axpsi^2+z2dmg^2);
  b = -2*(axpsi*adxpsi+z2dmg*z3d)*(adxpsi*z2dmg-z3d*axpsi);
  c = (axpsi^2+z2dmg^2)^2;
  theta2d = (a+b)/c;

  cphi = cos(phi);
  sphi = sin(phi);

  ctheta = cos(theta);
  stheta = sin(theta);

  p =  phid - stheta*psid;
  q =  cphi*thetad + sphi*ctheta*psid;
  r = -sphi*thetad + cphi*ctheta*psid;

  pd = phi2d - ctheta*thetad*psid - stheta*psi2d;
  a  = -sphi*phid*thetad + cphi*theta2d + cphi*ctheta*phid*psid;
  b  = -sphi*stheta*thetad*psid + sphi*ctheta*psi2d;
  qd =  a+b;
  a  = -cphi*phid*thetad - sphi*theta2d - sphi*ctheta*phid*psid;
  b  = -cphi*stheta*thetad*psid + cphi*ctheta*psi2d;
  rd = a+b;

  inp(2) = 1/DF_L*(DF_JXX*pd + (DF_JZZ-DF_JYY)*q*r);
  inp(3) = 1/DF_L*(DF_JYY*qd + (DF_JXX-DF_JZZ)*p*r);
  inp(4) = 1/DF_CM_OV_CT*(DF_JZZ*rd + (DF_JYY-DF_JXX)*p*q);

endfunction