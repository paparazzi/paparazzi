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

//fdm_Ct0 = 4. * fdm_mass * fdm_g / 2;   // thrust coefficient
//fdm_V0  = 1e9;	                       //
fdm_Cd  = 1e-9;                        // drag coefficient

fdm_min_thrust =  0.05; //  5%
fdm_max_thrust =  1.00; // 100%

fdm_wind = [0 0]';

fdm_T0 = 0.41*9.81; // Static maximum thrust = 0.41kg
fdm_Vlim = 15;  // Velocity of 0 thrust at full throttle in m/s
fdm_K = 3000*%pi/30/sqrt(0.3); // FIXME: Should be best fit of expression
                               // omega [rad/s] = K*sqrt(u) (a.k.a
                               // maximum rpm in rad/s)
fdm_propR = 4*0.0254;  // propeller radius in meters

// Don't change this //
Jlim = fdm_Vlim/(fdm_K*fdm_propR); // advance ratio at Vlim with full throttle
fdm_rhops = 1.25*fdm_K^2*%pi*fdm_propR^4; // rho * 'dynamic pressure' * surface
// Coeffs of CT(J) = aJ^2 + bJ + c
fdm_CTa = - 0.3173096; // From Qprop simulations with APC 8x3.8
fdm_CTc = 2*fdm_T0/fdm_rhops;
fdm_CTb = 1/Jlim*(-fdm_CTa*Jlim^2 - fdm_CTc);
// Now you can change //

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

  thrust = fdm_get_thrust(airspeed_body,X(FDM_STHETAD),U);

  lift_body = [0; sum(thrust)];
  lift_ltp = DCM'*lift_body;
  weight_ltp = [0; -fdm_g * fdm_mass];
  drag_ltp = -fdm_Cd * norm(airspeed_ltp) * airspeed_ltp;
  Xdot(FDM_SXD:FDM_SZD) = 1/fdm_mass*(lift_ltp+weight_ltp+drag_ltp);

  // moments
  Xdot(FDM_STHETAD) = fdm_la / fdm_inertia * (thrust(FDM_MOTOR_LEFT) - thrust(FDM_MOTOR_RIGHT));

  // add perturbation
  Xdot(FDM_SXD:FDM_STHETAD) = Xdot(FDM_SXD:FDM_STHETAD)+perturb;
endfunction

function thrust = fdm_get_thrust(airspeed_body,rates_body,U)

  vel_of_rate = [rates_body*fdm_la; -rates_body*fdm_la];

  for i = 1:FDM_MOTOR_NB
    adv(i) = (airspeed_body(FDM_SZ) + vel_of_rate(i))/(fdm_K*sqrt(U(i))*fdm_propR);
    c_t(i) = fdm_CTa*adv(i)^2 + fdm_CTb*adv(i) + fdm_CTc;
    if c_t(i) < 0
      c_t(i) = 0;
    end
    thrust(i) = U(i)*(0.5*fdm_rhops)*c_t(i);
  end

endfunction


