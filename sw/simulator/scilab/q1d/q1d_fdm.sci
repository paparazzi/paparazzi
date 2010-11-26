//
//  Flight dynamic model
//

// State Vector
FDM_Z    = 1;
FDM_ZD   = 2;
FDM_ZDD  = 3;
FDM_SIZE = 3;

// u

// Parameters
fdm_mass = 0.5;                           // mass in kg
fdm_g = 9.81;                             // gravity acceleration m/s2
fdm_max_thrust = 4.   * fdm_mass * fdm_g; // actuators high saturation
fdm_min_thrust = 0.05 * fdm_mass * fdm_g; // actuators low saturation
fdm_Cd   =  0.01;                         // non dimentional drag coefficient
fdm_Ct0  =   fdm_max_thrust;              // non dimentional lift coefficient
fdm_Ctzd =  -fdm_max_thrust/7;            // non dimensional lift coefficient

function [Xdot] = fdm_get_derivatives(t, X, U, perturb, param)

   mass = param(1);
   Xdot = zeros(FDM_ZD, 1);
   Xdot(FDM_Z)  = X(FDM_ZD);
   u_trim = trim(U(1), 0.01, 1.);
   thrust = u_trim * (fdm_Ct0 + fdm_Ctzd * X(FDM_ZD));
   Xdot(FDM_ZD) = 1/mass*(thrust - fdm_Cd * X(FDM_ZD) * abs(X(FDM_ZD))) - fdm_g + perturb;

endfunction

//
// Integrate the dynamic model
//
function  [Xfdmi1] = fdm_run(Xfdmi, Ui, ti, ti1, perturbi, parami)

  Xfdmi1 = zeros(FDM_SIZE,1);
  Xfdmi1(FDM_Z:FDM_ZD) = ode(Xfdmi(FDM_Z:FDM_ZD), ti, ti1, list(fdm_get_derivatives, Ui, perturbi, parami));
  xd = fdm_get_derivatives(ti1, Xfdmi1(FDM_Z:FDM_ZD), Ui, perturbi, parami);
  Xfdmi1(FDM_ZDD) = xd(FDM_ZD);
  Xfdmi1 = Xfdmi1;

endfunction

//
// Simple display
//
function fdm_display_simple(Xfdm, time)

nr = 3;
nc = 1;

subplot(nr,nc,1);
plot2d(time, Xfdm(FDM_Z,:));
xtitle('Z');

subplot(nr,nc,2);
plot2d(time, Xfdm(FDM_ZD,:));
xtitle('ZD');

subplot(nr,nc,3);
plot2d(time, Xfdm(FDM_ZDD,:));
xtitle('ZDD');

endfunction

