%
% 
%
%


function [sys,x0,str,ts] = range_meter_accel_kalman(t,x,u,flag)

period = 0.015625;
persistent X;      % state (Z, Zdot, Zdotdot)
persistent P;      % error covariance 

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
   X=[0. 0. 0.]';
   P=[1. 0. 0.
      0. 1. 0.
      0. 0. 1.];
   [sys,x0,str,ts]=mdlInitializeSizes(period);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,

   accel = u(1) - 9.81;
   rangemeter = u(2);
   % state transition model 
   F = [1. period       0.
        0.      1. period
        0.      0.      1. ];
   % control-input model
   B = [0];
   % process noise covariance
   Q = [period^4/4 period^3/2 period^2/2
        period^3/2 period^2   period
        period^2/2 period     1.];  

   % observation model
   H_a = [0 0 1];        
   % observation noise covariance
   R_a = [.01];

   % observation model
   H_r = [1 0 0];        
   % observation noise covariance
   R_r = [.0025];

   
   % predict
   X = F*X; % + B*u
   P = F*P*F'+ Q;
   
   % update rangemeter
   err = rangemeter - H_r*X;
   S = H_r*P*H_r' + R_r;
   K = P*H_r'*inv(S);
   X = X + err * K;
   P = (eye(3,3) - K*H_r)*P;

   % update accel
   err = accel - H_a*X;
   S = H_a*P*H_a' + R_a;
   K = P*H_a'*inv(S);
   X = X + err * K;
   P = (eye(3,3) - K*H_a)*P;
  
   
   
   sys=mdlOutputs(t,x,u, X);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes (period)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [period 0];

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)

sys = [];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u, X)


sys = [X(3) X(2) X(1)];

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
