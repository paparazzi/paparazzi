//
//
//
// Angle units
//
//
//
function [rad] = rad_of_deg(deg)
  rad = deg * %pi / 180; 
endfunction

function [deg] = deg_of_rad(rad)
  deg = rad * 180 / %pi;
endfunction

//
//
//
// Rotation convertion
//
//
//
function [quat] = quat_of_euler(euler) 
phi2   = euler(1) / 2.0;
theta2 = euler(2) / 2.0;
psi2   = euler(3) / 2.0;  
      			   
sinphi2 = sin( phi2 );   
cosphi2 = cos( phi2 );   
sintheta2 = sin( theta2 ); 
costheta2 = cos( theta2 ); 
sinpsi2   = sin( psi2 );   
cospsi2   = cos( psi2 );   

q0 =  cosphi2 * costheta2 * cospsi2 + sinphi2 * sintheta2 * sinpsi2;
q1 = -cosphi2 * sintheta2 * sinpsi2 + sinphi2 * costheta2 * cospsi2;
q2 =  cosphi2 * sintheta2 * cospsi2 + sinphi2 * costheta2 * sinpsi2;
q3 =  cosphi2 * costheta2 * sinpsi2 - sinphi2 * sintheta2 * cospsi2;

quat = [q0 q1 q2 q3]';

endfunction


function [euler] = euler_of_quat(quat)
  dcm00 = 1.0 - 2*(quat(3)*quat(3) + quat(4)*quat(4));
  dcm01 =       2*(quat(2)*quat(3) + quat(1)*quat(4));
  dcm02 =       2*(quat(2)*quat(4) - quat(1)*quat(3));
  dcm12 =       2*(quat(3)*quat(4) + quat(1)*quat(2));
  dcm22 = 1.0 - 2*(quat(2)*quat(2) + quat(3)*quat(3));

  phi = atan( dcm12, dcm22 );
  theta = -asin( dcm02 );
  psi = atan( dcm01, dcm00 );

  euler = [phi; theta; psi];
endfunction


function [DCM] = dcm_of_euler(euler)
  phi = euler(1);
  theta = euler(2);
  psi = euler(3);
  dcm11 = cos(theta)*cos(psi);
  dcm12 = cos(theta)*sin(psi);
  dcm13 = -sin(theta);
  dcm21 = sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi);
  dcm22 = sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi);
  dcm23 = sin(phi)*cos(theta);
  dcm31 = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
  dcm32 = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
  dcm33 = cos(phi)*cos(theta);
  DCM= [ dcm11 dcm12 dcm13
         dcm21 dcm22 dcm23
	 dcm31 dcm32 dcm33 ];
endfunction
   
function [DCM] = dcm_of_quat(quat)

q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);

dcm00 = q0^2 + q1^2 - q2^2 - q3^2;
dcm01 = 2 * (q1*q2 + q0*q3);
dcm02 = 2 * (q1*q3 - q0*q2);
dcm10 = 2 * (q1*q2 - q0*q3);
dcm11 = q0^2 - q1^2 + q2^2 - q3^2;
dcm12 = 2 * (q2*q3 + q0*q1);
dcm20 = 2 * (q1*q3 + q0*q2);
dcm21 = 2 * (q2*q3 - q0*q1);
dcm22 = q0^2 - q1^2 - q2^2 + q3^2;

DCM = [ dcm00 dcm01 dcm02
        dcm10 dcm11 dcm12
        dcm20 dcm21 dcm22 ];

endfunction


function [X1] = normalise_quat(X0)
 quat = X0(1:4);
 quat = quat / norm(quat);
 X1 = [quat];
endfunction

