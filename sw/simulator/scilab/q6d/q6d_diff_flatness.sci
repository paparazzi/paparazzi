
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
    
  axpsi = cos(state(DF_REF_PSI))*fo(1,3) + sin(state(DF_REF_PSI))*fo(2,3);
  aypsi = sin(state(DF_REF_PSI))*fo(1,3) - cos(state(DF_REF_PSI))*fo(2,3);
  zddmg = fo(3,3) - DF_G;
  av = sqrt(axpsi^2 + zddmg^2);
  
  state(DF_REF_PHI) = atan(aypsi/av);
  state(DF_REF_THETA) = atan(axpsi/zddmg);
  
  jxpsi = cos(state(DF_REF_PSI))*fo(1,4) + sin(state(DF_REF_PSI))*fo(2,4);
  jypsi = sin(state(DF_REF_PSI))*fo(1,4) - cos(state(DF_REF_PSI))*fo(2,4);
   
  kxpsi = cos(state(DF_REF_PSI))*fo(1,5) + sin(state(DF_REF_PSI))*fo(2,5);
  kypsi = sin(state(DF_REF_PSI))*fo(1,5) - cos(state(DF_REF_PSI))*fo(2,5);

  psid = fo(4,2);
  
  adxpsi = -psid*aypsi + jxpsi;
  adypsi =  psid*axpsi + jypsi;

  adv = (axpsi*adxpsi + zddmg*fo(3,4))/av;
  
  phid   = (adypsi*av-adv*aypsi)/(aypsi^2+av^2);
  thetad = (adxpsi*zddmg-fo(3,4)*aypsi)/(axpsi^2+zddmg^2);
  
  state(DF_REF_P) =  phid - sin(state(DF_REF_THETA))*psid;
  state(DF_REF_Q) =  cos(state(DF_REF_PHI))*thetad +   sin(state(DF_REF_PHI))*cos(state(DF_REF_THETA))*psid;
  state(DF_REF_R) = -sin(state(DF_REF_PHI))*thetad +   cos(state(DF_REF_PHI))*cos(state(DF_REF_THETA))*psid;
  
endfunction

// control input from flat output
function [inp] = df_input_of_fo(fo)

  inp = zeros(4,1);
  
  xdd = fo(1,3);
  ydd = fo(2,3);
  zddmg = fo(3,3) - DF_G;
  inp(1) = DF_MASS * sqrt(xdd^2+ydd^2+zddmg^2);
  
  psi =  fo(4,1);
  
  axpsi = cos(psi)*xdd + sin(psi)*ydd;
  aypsi = sin(psi)*xdd - cos(psi)*ydd;
  zddmg = fo(3,3) - DF_G;
  av = sqrt(axpsi^2 + zddmg^2);
  
  a = (addypsi*av + adv*(adypsi-aypsi)-addv*aypsi)*(aypsi^2+av^2);
  b = -2*(aypsi*adypsi+av*adv)*(adypsi*av-adv*aypsi);
  c = (aypsi^2+av^2)^2;
  phidd = (a + b)/c;
  
  a = (addxpsi*zddmg+fo(3,4)*(adxpsi - axpsi) - fo(3,5)*axpsi)*(axpsi^2+zddmg^2);
  b = -2*(axpsi*adxpsi+zddmg*fo(3,4))*(adxpsi*zddmg-fo(3,4)*axpsi);
  c = (axpsi^2+zddmg^2)^2;
  thetadd = (a+b)/c;
   
  psidd = fo(4,3);
  
  
  
endfunction