//
//
//
// Initialisation
//
//
//
function [X0] = ahrs_euler_init(avg_len, accel, mag, gyro) 

  avg_accel = sum(accel(:,1:avg_len), 'c') / avg_len;
  avg_mag = sum(mag(:,1:avg_len), 'c') / avg_len;
  avg_gyro = sum(gyro(:,1:avg_len), 'c') / avg_len;

  phi0 = phi_of_accel(avg_accel);
  theta0 = theta_of_accel(avg_accel);
  psi0 = psi_of_mag(phi0, theta0, avg_mag);


  X0 = [ phi0 theta0 psi0 avg_gyro(1) avg_gyro(2) avg_gyro(3) ]';

endfunction

//
//
//
// Filter 
//
//
//

function [Pi] = ahrs_euler_get_P(P, i)
  Pi = P(:, 1+6*(i-1):6*i);
endfunction

//
//
//
// Display 
//
//
//
function [] = ahrs_euler_display(time, X, P, M)
  xbasc();
  subplot(3,1,1)
  xtitle('Angle');
  EstAngleDeg = X(1:3,:) * 180 / %pi;
  MeasAngleDeg = M * 180 / %pi;
  plot2d([time; time; time]', EstAngleDeg', style=[5, 3, 2], leg="phi@theta@psi");
  plot2d([time; time; time]', MeasAngleDeg', style=[0, 0, 0]);
  
  subplot(3,1,2)
  xtitle('Bias');
  EstBiasDeg = X(4:6,:) * 180 / %pi;
  plot2d([time; time; time]', EstBiasDeg', style=[5, 3, 2], leg="phi@theta@psi");

  subplot(3,1,3)
  xtitle('Covariance');
  
  Pdiag = [];
  for i=1:length(time)
    Pi = ahrs_euler_get_P(P, i);
    Pdiag = [Pdiag [Pi(1,1) Pi(2,2) Pi(3,3) Pi(4,4) Pi(5,5) Pi(6,6)]'];
  end
  plot2d([time; time; time; time; time; time]', Pdiag', style=[5, 3, 2, 0, 0, 0],... 
      leg="phi@theta@psi@bp@bq@br");

  
endfunction

//
//
//
// Time derivative of state
//
//
//
function [Xdot] = ahrs_euler_get_Xdot(X, rate)

  phi   = X(1);
  theta = X(2);
  psi   = X(3);
  
  euler_dot = [ 1  sin(phi)*tan(theta)   cos(phi)*tan(theta)
                0  cos(phi)             -sin(phi)
                0  sin(phi)/cos(theta)   cos(phi)/cos(theta)
	      ] * rate;
  Xdot = [ euler_dot; 0; 0; 0];
	  	  
endfunction


//
//
//
// Derivative of Xdot wrt X
//
//
//
function [F] = ahrs_euler_get_F(X, rate)
  phi   = X(1);
  theta = X(2);
  psi   = X(3);
  
  p = rate(1);
  q = rate(2);
  r = rate(3);

  d_phidot_dX =  [ cos(phi)*tan(theta)*q - sin(phi)*tan(theta)*r
                          1/(1+theta^2) * (sin(phi)*q+cos(phi)*r)
                     0
                    -1
                    -sin(phi)*tan(theta)
                    -cos(phi)*tan(theta) ]';

  d_thetadot_dX = [ -sin(phi)*q - cos(phi)*r
                        0
                        0
                        0
                       -cos(phi)
                        sin(phi) ]';

  d_psidot_dX = [ cos(phi)/cos(theta)*q - sin(phi)/cos(theta)*r
                  sin(theta)/cos(theta)^2*(sin(phi)*q+cos(phi)*r)
                  0
                  0
                  -sin(phi)/cos(theta)
                  -cos(phi)/cos(theta)
                 ]';

  F = [ d_phidot_dX
        d_thetadot_dX
        d_psidot_dX
        0 0 0 0 0 0
        0 0 0 0 0 0
        0 0 0 0 0 0
      ];
 
endfunction



//
//
//
// Derivative of measure wrt state 
//
//
//
function [H] = ahrs_euler_compm_get_deuler_dX(X)

H = [ 
      1 0 0 0 0 0
      0 1 0 0 0 0
      0 0 1 0 0 0
    ];

endfunction
