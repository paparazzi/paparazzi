

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
// Initialisation
//
//
//
function [X0] = ahrs_quat_init(avg_len, accel, mag, gyro) 

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
// Filter 
//
//
//






//
//
//
// Display 
//
//
//


//
//
//
// Derivative of measure wrt state 
//
//
//
function [H] = ahrs_quat_get_dphi_dq(quat)

DCM = dcm_of_quat(quat);
phi_err = 2 / (DCM(3,3)^2 + DCM(2,3)^2);
q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);
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


function  [H] = ahrs_quat_get_dtheta_dq(quat)

DCM = dcm_of_quat(quat);
theta_err = 2 / sqrt(1 - DCM(1,3)^2);
q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);
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

function  [H] = ahrs_quat_get_dpsi_dq(quat)

DCM = dcm_of_quat(quat);
psi_err = 2 / (DCM(1,1)^2 + DCM(1,2)^2);
q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);
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