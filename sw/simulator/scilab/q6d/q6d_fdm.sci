//
// Flight Dynamic Model of a quadrotor
//


//
// State Vector
//
FDM_SX    =  1;
FDM_SY    =  2;
FDM_SZ    =  3;
FDM_SXD   =  4;
FDM_SYD   =  5;
FDM_SZD   =  6;
FDM_SQI   =  7;
FDM_SQX   =  8;
FDM_SQY   =  9;
FDM_SQZ   = 10;
FDM_SP    = 11;
FDM_SQ    = 12;
FDM_SR    = 13;
FDM_SSIZE = 13;

//
// Actuators
//
FDM_MOTOR_FRONT = 1;
FDM_MOTOR_RIGHT = 2;
FDM_MOTOR_BACK  = 3;
FDM_MOTOR_LEFT  = 4;
FDM_MOTOR_NB    = 4;


//
// By Products
//
FDM_EPHI   = 1;  // euler angles
FDM_ETHETA = 2;
FDM_EPSI   = 3;

FDM_AXDD   = 1;  // linear accelerations
FDM_AYDD   = 2;
FDM_AZDD   = 3;

FDM_RAPD   = 1;  // rotational accelerations
FDM_RAQD   = 2;
FDM_RARD   = 3;


fdm_g = 9.81;                          // gravitational acceleration
fdm_mass = 0.5;                        // mass in kg
fdm_inertia = [0.0078 0.     0.        // inertia tensor
               0.     0.0078 0.
	       0.     0.     0.0137 ];
fdm_Ct0 = 4. * fdm_mass * fdm_g / 4;   // thrust coefficient
//fdm_V0  = 7.;	                       //
fdm_V0  = 1e6;	                       //
//fdm_Cd  = 0.01;                      // drag coefficient
fdm_Cd  = 1e-6;                        // drag coefficient
fdm_la  = 0.25;                        // arm length
fdm_Cm  = fdm_Ct0 / 10;                // torque coefficient

fdm_wind = [ 0; 0; 0];

fdm_dt = 1/512;

fdm_max_u = 1.0; // 100%
fdm_min_u = 0.1; // 10%

global fdm_time;
global fdm_state;
global fdm_euler;
global fdm_accel;
global fdm_raccel;


function fdm_init(time, X0)

  global fdm_time;
  fdm_time = time;
  global fdm_state;
  fdm_state = zeros(FDM_SSIZE, length(fdm_time));
  fdm_state(:,1) = X0;
  global fdm_euler;
  fdm_euler = zeros(AXIS_NB, length(fdm_time));
  fdm_euler(:,1) = euler_of_quat(fdm_state(FDM_SQI:FDM_SQZ,1));
  global fdm_accel;
  fdm_accel = zeros(AXIS_NB, length(fdm_time));
  global fdm_raccel;
  fdm_raccel = zeros(AXIS_NB, length(fdm_time));

endfunction



function fdm_run(i, cmd)

  cmd = trim_vect(cmd,fdm_min_u, fdm_max_u);
  global fdm_state;
  global fdm_time;
  fdm_state(:,i) = ode(fdm_state(:,i-1), fdm_time(i-1), fdm_time(i), list(fdm_get_derivatives, cmd));
  xd = fdm_get_derivatives(fdm_time(i), fdm_state(:, i), cmd);
  global fdm_accel;
  fdm_accel(:,i) = xd(FDM_SXD:FDM_SZD);
  global fdm_raccel;
  fdm_raccel(:,i) =  xd(FDM_SP:FDM_SR);
  global fdm_euler;
  fdm_euler(:,i) = euler_of_quat(fdm_state(FDM_SQI:FDM_SQZ,i));

endfunction



function [Xdot] = fdm_get_derivatives(t, X, U)

  Xdot = zeros(length(X),1);
  // position
  Xdot(FDM_SX:FDM_SZ) = X(FDM_SXD:FDM_SZD);
  // speed
  Xdot(FDM_SXD:FDM_SZD) = 1/fdm_mass * fdm_get_forces_ltp(X, U);
  // orientation quaternion
  rates_body = X(FDM_SP:FDM_SR);
  W = get_omega_quat(rates_body);
  K_lagrange = 1.;
  quat = X(FDM_SQI:FDM_SQZ);
  quat_dot =  -1/2 * W * quat;
  quat_dot = quat_dot + K_lagrange * ( 1 - norm(quat)) * quat;
  Xdot(FDM_SQI:FDM_SQZ) = quat_dot;
  // body rates
  moments_body = fdm_get_moments_body(X, U);
  i_omega = fdm_inertia * rates_body;
  omega_i_omega = cross_product(rates_body, i_omega);
  Xdot(FDM_SP:FDM_SR) = inv(fdm_inertia) * (moments_body - omega_i_omega);

endfunction


function [F_ltp] = fdm_get_forces_ltp(X, U)

  gspeed_ltp =  X(FDM_SXD:FDM_SZD);
  airspeed_ltp = gspeed_ltp - fdm_wind;
  quat = X(FDM_SQI:FDM_SQZ);
  airspeed_body = quat_vect_mult(quat, airspeed_ltp);
  lift_body = [0; 0; -sum(U) * fdm_Ct0 * ( 1 - abs(1/fdm_V0 * airspeed_body(AXIS_Z)))];
  lift_ltp = quat_vect_inv_mult(quat, lift_body);

  weight_ltp = [0; 0; fdm_g * fdm_mass];

  drag_ltp = -fdm_Cd * norm(airspeed_ltp) * airspeed_ltp;

  F_ltp = lift_ltp + weight_ltp + drag_ltp;

endfunction


function [M_body] = fdm_get_moments_body(X, U)

  k1 = fdm_la * fdm_Ct0;
  k2 = fdm_Cm;
  moments_of_u = [ 0  -k1  0  k1
                   k1  0  -k1 0
		  -k2  k2 -k2 k2 ];

  M_body = moments_of_u * U;

endfunction


