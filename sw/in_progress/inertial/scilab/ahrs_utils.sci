

//
//
//
// Angles Measurements
//
//
//
function [phi] = phi_of_accel(accel)
  phi = atan(accel(2), accel(3));
endfunction

function [theta] = theta_of_accel(accel)
  theta = -asin(accel(1) / norm(accel));   
endfunction

function [psi] = psi_of_mag(phi, theta, mag)
  cphi   = cos( phi );
  sphi   = sin( phi );
  ctheta = cos( theta );
  stheta = sin( theta );
  mn = ctheta*      mag(1)+ sphi*stheta* mag(2)+ cphi*stheta* mag(3);
  me = cphi*  mag(2) -sphi* mag(3);
  psi = -atan( me, mn );
endfunction

function [m_eulers] = ahrs_compute_euler_measurements(accel, mag)
  m_eulers = [];
  [m n] = size(accel);
  for i=1:n
    phi = phi_of_accel(accel(:,i));
    theta = theta_of_accel(accel(:,i));
    psi = psi_of_mag(phi, theta, mag(:,i));
    m_eulers = [m_eulers [phi;theta;psi]];
  end

endfunction

//
//
//
// Angles convertion
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

quat = [q0 q1 q2 q3];

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

  euler = [phi theta psi];
endfunction

function [X1] = normalise_quat(X0)
 quat = X0(1:4);
 quat = quat / norm(quat);
 X1 = [quat; X0(5:7)];
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

//
//
//
// Initialisation
//
//
//
function [X0] = ahrs_init(avg_len, accel, mag, gyro) 

avg_accel = sum(accel(:,1:avg_len), 'c') / avg_len;
avg_mag = sum(mag(:,1:avg_len), 'c') / avg_len;
avg_gyro = sum(gyro(:,1:avg_len), 'c') / avg_len;

phi0 = phi_of_accel(avg_accel);
theta0 = theta_of_accel(avg_accel);
psi0 = psi_of_mag(phi0, theta0, avg_mag);

[quat] = quat_of_euler([phi0, theta0, psi0])

X0 = [ quat avg_gyro(1) avg_gyro(2) avg_gyro(3) ]';

endfunction


//
//
//
// 
//
//
//
