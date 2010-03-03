//
// Flight Dynamic Model of a planar quadrotor
//

//
// State Vector
//
FDM_SX      = 1;
FDM_SZ      = 2;
FDM_STHETA  = 3;
FDM_SXD     = 4;
FDM_SZD     = 5;
FDM_STHETAD = 6;
FDM_SSIZE   = 6;

//
// Accelerations
//
FDM_AX     = 1;
FDM_AZ     = 2;
FDM_ATHETA = 3;
FDM_ASIZE  = 3;

//
// Actuators
//
FDM_MOTOR_RIGHT = 1;
FDM_MOTOR_LEFT  = 2;
FDM_MOTOR_NB    = 2;

fdm_g       = 9.81;
fdm_mass    = 0.25;
fdm_inertia = 0.0078;
fdm_la  = 0.25;                        // arm length

fdm_Ct0 = 4. * fdm_mass * fdm_g / 2;   // thrust coefficient
fdm_V0  = 1e9;	                       // 
fdm_Cd  = 1e-9;                        // drag coefficient

fdm_min_thrust =  0.05; //  5%  
fdm_max_thrust =  1.00; // 100%

fdm_wind = [0 0]';

global fdm_time;
global fdm_state;                    // state
global fdm_accel;                    // aceleration (used for sensors)
global fdm_perturb;                  // perturbation

function fdm_init(time, fdm_X0, cmd0) 

  global fdm_time;
  fdm_time = time;
  global fdm_state;
  fdm_state = zeros(FDM_SSIZE, length(fdm_time));
  fdm_state(:,1) = fdm_X0;
  global fdm_accel;
  fdm_accel = zeros(FDM_ASIZE, length(fdm_time));
  accel = fdm_get_derivatives(time(1), fdm_state(:,1), cmd0, [0;0;0]);
  fdm_accel(:,1) = accel(FDM_SXD:FDM_STHETAD);
  global fdm_perturb;
  fdm_perturb = zeros(FDM_ASIZE, length(fdm_time));
endfunction


// propagate dynamic model from step i-1 to step i
function fdm_run(i, cmd)
 
  cmd = trim_vect(cmd, fdm_min_thrust, fdm_max_thrust);
  global fdm_state;
//  global fdm_time;
  fdm_state(:,i) = ode(fdm_state(:,i-1), fdm_time(i-1), fdm_time(i), list(fdm_get_derivatives, cmd, fdm_perturb(:,i-1)));
  global fdm_accel;
  accel = fdm_get_derivatives(fdm_time(i), fdm_state(:,i), cmd, fdm_perturb(:,i));
  fdm_accel(:,i) = accel(FDM_SXD:FDM_STHETAD);

endfunction

function [Xdot] = fdm_get_derivatives(t, X, U, perturb)

  Xdot = zeros(length(X),1);
  
  Xdot(FDM_SX:FDM_STHETA) = X(FDM_SXD:FDM_STHETAD);
  
  // forces :
  gspeed_ltp =  X(FDM_SXD:FDM_SZD);
  airspeed_ltp = gspeed_ltp - fdm_wind;
   
  stheta = sin(X(FDM_STHETA));
  ctheta = cos(X(FDM_STHETA));
  
  DCM = [ctheta stheta ; -stheta ctheta];
  airspeed_body = DCM * airspeed_ltp;
  
  lift_body = [0; sum(U) * fdm_Ct0 * ( 1 - abs(1/fdm_V0 * airspeed_body(AXIS_Z)))];
  lift_ltp = DCM'*lift_body; 
  weight_ltp = [0; -fdm_g * fdm_mass];
  drag_ltp = -fdm_Cd * norm(airspeed_ltp) * airspeed_ltp;
  Xdot(FDM_SXD:FDM_SZD) = 1/fdm_mass*(lift_ltp+weight_ltp+drag_ltp);
  
  // moments
  Xdot(FDM_STHETAD) = fdm_la * fdm_Ct0 / fdm_inertia*(U(FDM_MOTOR_LEFT) - U(FDM_MOTOR_RIGHT));
  
  // add perturbation
  Xdot(FDM_SXD:FDM_STHETAD) = Xdot(FDM_SXD:FDM_STHETAD)+perturb;
endfunction

