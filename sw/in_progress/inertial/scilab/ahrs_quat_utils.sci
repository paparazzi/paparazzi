




//
//
//
// Initialisation
//
//
//
function [X0] = ahrs_quat_init(avg_len, accel, mag, gyro) 

 [AE0] = ahrs_euler_init(avg_len, accel, mag, gyro);

 [quat] = quat_of_euler(AE0(1:3));

 X0 = [ quat AE0(4:6) ]';

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