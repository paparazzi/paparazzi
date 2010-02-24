
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
function [state] = df_state_of_fo(fo)

  state = zeros(DF_REF_SIZE, 1);
  state(DF_REF_X)      = fo(1,1);
  state(DF_REF_Y)      = fo(2,1);
  state(DF_REF_Z)      = fo(3,1);

  state(DF_REF_XD)     = fo(1,2);
  state(DF_REF_YD)     = fo(2,2);
  state(DF_REF_ZD)     = fo(3,2);

  state(DF_REF_PSI)    = fo(4,1);
  
  psi = state(DF_REF_PSI);
  cpsi = cos(psi);
  spsi = sin(psi);
  
  x2d = fo(1,3);
  y2d = fo(2,3);
  z2d = fo(2,3);
  
  axpsi = cpsi*x2d + spsi*y2d;
  aypsi = spsi*x2d - cpsi*y2d;
  z2dmg = z2d - DF_G;
  av = sqrt(axpsi^2 + z2dmg^2);
  
  state(DF_REF_PHI) = atan(aypsi/av);
  state(DF_REF_THETA) = atan(axpsi/z2dmg);
  
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
  
  phid   = (adypsi*av-adv*aypsi)/(aypsi^2+av^2);
  thetad = (adxpsi*z2dmg-z3d*aypsi)/(axpsi^2+z2dmg^2);
  
  cphi = cos(state(DF_REF_PHI));
  sphi = sin(state(DF_REF_PHI));

  ctheta = cos(state(DF_REF_THETA));
  stheta = sin(state(DF_REF_THETA));
  
  state(DF_REF_P) =  phid - stheta*psid;
  state(DF_REF_Q) =  cphi*thetad + sphi*ctheta*psid;
  state(DF_REF_R) = -sphi*thetad + cphi*ctheta*psid;
  
endfunction




// control input from flat output
function [inp] = df_input_of_fo(fo)

  inp = zeros(4,1);
  
  xdd = fo(1,3);
  ydd = fo(2,3);
  zddmg = fo(3,3) - DF_G;
  inp(1) = DF_MASS * sqrt(xdd^2+ydd^2+zddmg^2);
  
  psi   = fo(4,1);
  psid  = fo(4,2);
  psidd = fo(4,3);
   
  axpsi = cos(psi)*xdd + sin(psi)*ydd;
  aypsi = sin(psi)*xdd - cos(psi)*ydd;
  
  xddd = fo(1,4);
  yddd = fo(2,4);
  
  jxpsi = cos(psi)*xddd + sin(psi)*yddd;
  jypsi = sin(psi)*xddd - cos(psi)*yddd;
  
  xdddd = fo(1,5);
  ydddd = fo(2,5);
  
  kxpsi = cos(psi)*xdddd + sin(psi)*ydddd;
  kypsi = sin(psi)*xdddd - cos(psi)*ydddd;
  
  adxpsi = -psid*aypsi + jxpsi;
  adypsi =  psid*axpsi + jypsi;
  
  addxpsi = -psidd*aypsi - psid^2*axpsi - 2*psid*jypsi + kxpsi;
  addypsi =  psidd*axpsi - psid^2*aypsi + 2*psid*jxpsi + kypsi;
  
  av = sqrt(axpsi^2 + zddmg^2);
  zddd = fo(3,4);
  adv = (axpsi*adxpsi+zddmg*zddd)/av;
  zdddd = fo(3,5);
  a = (axpsi*addxpsi + adxpsi^2 + (zddmg)*zdddd +zddd)*av;
  b = -adv*(axpsi*adxpsi + zddmg*zddd);
  addv = (a+b)/av^2;
  
  phi = atan(aypsi/av);
  theta = atan(axpsi/zddmg);
  
  phid   = (adypsi*av-adv*aypsi)/(aypsi^2+av^2);
  thetad = (adxpsi*zddmg-zddd*aypsi)/(axpsi^2+zddmg^2);
    
  a = (addypsi*av + adv*(adypsi-aypsi)-addv*aypsi)*(aypsi^2+av^2);
  b = -2*(aypsi*adypsi+av*adv)*(adypsi*av-adv*aypsi);
  c = (aypsi^2+av^2)^2;
  phidd = (a+b)/c;
  
  a = (addxpsi*zddmg+fo(3,4)*(adxpsi - axpsi) - fo(3,5)*axpsi)*(axpsi^2+zddmg^2);
  b = -2*(axpsi*adxpsi+zddmg*fo(3,4))*(adxpsi*zddmg-fo(3,4)*axpsi);
  c = (axpsi^2+zddmg^2)^2;
  thetadd = (a+b)/c;
   
  p =  phid - sin(theta)*psid;
  q =  cos(phi)*thetad + sin(phi)*cos(theta)*psid;
  r = -sin(phi)*thetad + cos(phi)*cos(theta)*psid;
  
  pd = phidd - cos(theta)*thetad*psid - sin(theta)*psidd;
  a  = -sin(phi)*phid*thetad + cos(phi)*thetadd + cos(phi)*cos(theta)*phid*psid;
  b  = -sin(phi)*sin(theta)*thetad*psid + sin(phi)*cos(theta)*psidd;
  qd =  a+b;
  a  = -cos(phi)*phid*thetad - sin(phi)*thetadd - sin(phi)*cos(theta)*phid*psid;
  b  = -cos(phi)*sin(theta)*thetad*psid + cos(phi)*cos(theta)*psidd; 
  rd = a+b;
  
  inp(2) = DF_JXX/DF_L*pd + (DF_JZZ-DF_JYY)*q*r;
  inp(3) = DF_JYY/DF_L*qd + (DF_JXX-DF_JZZ)*p*r;
  inp(4) = DF_JZZ/DF_CM_OV_CT*rd + (DF_JYY-DF_JXX)*p*q;
  
endfunction