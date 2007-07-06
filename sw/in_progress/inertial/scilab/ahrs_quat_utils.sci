//
//
//
// Initialisation
//
//
//
function [X0] = ahrs_quat_init(avg_len, accel, mag, gyro) 

 [AEX0] = ahrs_euler_init(avg_len, accel, mag, gyro);

 [quat] = quat_of_euler(AEX0(1:3));

 X0 = [ quat; AEX0(4:6) ];

endfunction


//
//
//
// Filter 
//
//
//

function [Pi] = ahrs_quat_get_P(P, i)
  Pi = P(:, 1+7*(i-1):7*i);
endfunction


//
//
//
// Time derivative of state
//
//
//
function [Xdot] = ahrs_quat_get_Xdot(X, rate)
  p = rate(1);
  q = rate(2);
  r = rate(3);
  OMEGA =   1/2 * [ 0  -p  -q  -r  0  0  0
                    p   0   r  -q  0  0  0
                    q  -r   0   p  0  0  0
                    r   q  -p   0  0  0  0
		    0   0   0   0  0  0  0
		    0   0   0   0  0  0  0
		    0   0   0   0  0  0  0 ];
  Xdot = OMEGA * X;
endfunction


//
//
//
// Derivative of Xdot wrt X
//
//
//
function [F] = ahrs_quat_get_F(X, rate)
  q0 = X(1);
  q1 = X(2);
  q2 = X(3);
  q3 = X(4);

  p = rate(1);
  q = rate(2);
  r = rate(3);

  F =   1/2 * [ 0  -p  -q  -r   q1  q2  q3;
                p   0   r  -q  -q0  q3 -q2;
                q  -r   0   p  -q3  q0  q1;
                r   q  -p   0   q2 -q1 -q0;
                0   0   0   0    0   0   0;
                0   0   0   0    0   0   0;
                0   0   0   0    0   0   0 ];
	  
endfunction


//
//
//
// Derivative of measure wrt state 
//
//
//
function [H] = ahrs_quat_get_dphi_dX(X)

DCM = dcm_of_quat(X(1:4));
phi_err = 2 / (DCM(3,3)^2 + DCM(2,3)^2);
q0 = X(1);
q1 = X(2);
q2 = X(3);
q3 = X(4);
H = [
    (q1 * DCM(3,3))                     * phi_err
    (q0 * DCM(3,3) + 2 * q1 * DCM(2,3)) * phi_err
    (q3 * DCM(3,3) + 2 * q2 * DCM(2,3)) * phi_err
    (q2 * DCM(3,3))                     * phi_err
    0
    0
    0
    ]';

endfunction

function  [H] = ahrs_quat_get_dtheta_dX(X)

DCM = dcm_of_quat(X(1:4));
theta_err = 2 / sqrt(1 - DCM(1,3)^2);
q0 = X(1);
q1 = X(2);
q2 = X(3);
q3 = X(4);
H = [
     q2 * theta_err
    -q3 * theta_err
     q0 * theta_err
    -q1 * theta_err
     0
     0
     0
    ]';

endfunction

function  [H] = ahrs_quat_get_dpsi_dX(X)

DCM = dcm_of_quat(X(1:4));
psi_err = 2 / (DCM(1,1)^2 + DCM(1,2)^2);
q0 = X(1);
q1 = X(2);
q2 = X(3);
q3 = X(4);
H = [
    (q3 * DCM(1,1))                     * psi_err
    (q2 * DCM(1,1))                     * psi_err
    (q1 * DCM(1,1) + 2 * q2 * DCM(1,2)) * psi_err
    (q0 * DCM(1,1) + 2 * q3 * DCM(1,2)) * psi_err
    0
    0
    0
    ]';

endfunction

function [H] = ahrs_quat_compm_get_deuler_dX(X)

Hphi =  ahrs_quat_get_dphi_dX(X);
Htheta =  ahrs_quat_get_dtheta_dX(X);
Hpsi =  ahrs_quat_get_dpsi_dX(X);
H = [ Hphi; Htheta; Hpsi];

endfunction

//
//
//
// Display 
//
//
//
function [] = ahrs_quat_display(time, X, P, M)
  xbasc();
  subplot(3,1,1)
  xtitle('Angle');
  EstEulerDeg = [];
  for i=1:length(time)
    ead = euler_of_quat(X(1:4, i)) * 180 / %pi;
    EstEulerDeg = [EstEulerDeg  ead];
  end  
  MeasAngleDeg = M * 180 / %pi;
  plot2d([time; time; time]', EstEulerDeg', style=[5, 3, 2], leg="phi@theta@psi");
  plot2d([time; time; time]', MeasAngleDeg', style=[0, 0, 0]);
  
  subplot(3,1,2)
  xtitle('Bias');
  EstBiasDeg = X(5:7,:) * 180 / %pi;
  plot2d([time; time; time]', EstBiasDeg', style=[5, 3, 2], leg="phi@theta@psi");

  subplot(3,1,3)
  xtitle('Covariance');
  
  Pdiag = [];
  for i=1:length(time)
    Pi = ahrs_quat_get_P(P, i);
    Pdiag = [Pdiag [Pi(1,1) Pi(2,2) Pi(3,3) Pi(4,4) Pi(5,5) Pi(6,6) Pi(7,7)]'];
  end
  plot2d([time; time; time; time; time; time; time]', Pdiag', style=[5, 4, 3, 2, 0, 0, 0],... 
      leg="q0@q1@q2@q3@bp@bq@br");

endfunction

