function [o] = trim(i,_min, _max)
  o = i;
  if i > _max
    o = _max
  elseif i < _min
    o = _min
  end
endfunction

function [o] = trim_vect(i,_min, _max)
  o = i;
  for j=1:length(i)
    if i(j) > _max
      o(j) = _max
    elseif i(j) < _min
      o(j) = _min
    end
  end
endfunction



function [rad] = rad_of_deg(deg)
  rad = deg / 180 * %pi;
endfunction

function [deg] = deg_of_rad(rad)
  deg = rad * 180 / %pi;
endfunction


EULER_PHI   = 1;
EULER_THETA = 2;
EULER_PSI   = 3;
EULER_NB    = 3;

AXIS_X  = 1;
AXIS_Y  = 2;
AXIS_Z  = 3;
AXIS_NB = 3;


Q_QI = 1;
Q_QX = 2;
Q_QY = 3;
Q_QZ = 4;
Q_SIZE = 4;


//
//
//
function [quat] = quat_null()
  quat = [1 0 0 0]';
endfunction

//
//
//
function [dcm] = dcm_of_quat(q)

  qi2  = q(Q_QI)*q(Q_QI);
  qiqx = q(Q_QI)*q(Q_QX);
  qiqy = q(Q_QI)*q(Q_QY);
  qiqz = q(Q_QI)*q(Q_QZ);
  qx2  = q(Q_QX)*q(Q_QX);
  qxqy = q(Q_QX)*q(Q_QY);
  qxqz = q(Q_QX)*q(Q_QZ);
  qy2  = q(Q_QY)*q(Q_QY);
  qyqz = q(Q_QY)*q(Q_QZ);
  qz2  = q(Q_QZ)*q(Q_QZ);
  m00 = qi2 + qx2 - qy2 - qz2;
  m01 = 2 * ( qxqy + qiqz );
  m02 = 2 * ( qxqz - qiqy );
  m10 = 2 * ( qxqy - qiqz );
  m11 = qi2 - qx2 + qy2 - qz2;
  m12 = 2 * ( qyqz + qiqx );
  m20 = 2 * ( qxqz + qiqy );
  m21 = 2 * ( qyqz - qiqx );
  m22 = qi2 - qx2 - qy2 + qz2;
  dcm = [ m00 m01 m02
          m10 m11 m12
	  m20 m21 m22 ];

endfunction


//
//
//
function [quat] = quat_of_euler(euler)

  phi2   = euler(EULER_PHI) / 2.0;
  theta2 = euler(EULER_THETA) / 2.0;
  psi2   = euler(EULER_PSI) / 2.0;

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

//
//
//
function [euler] = euler_of_quat(quat)
  dcm00 = 1.0 - 2*(quat(Q_QY)*quat(Q_QY) + quat(Q_QZ)*quat(Q_QZ));
  dcm01 =       2*(quat(Q_QX)*quat(Q_QY) + quat(Q_QI)*quat(Q_QZ));
  dcm02 =       2*(quat(Q_QX)*quat(Q_QZ) - quat(Q_QI)*quat(Q_QY));
  dcm12 =       2*(quat(Q_QY)*quat(Q_QZ) + quat(Q_QI)*quat(Q_QX));
  dcm22 = 1.0 - 2*(quat(Q_QX)*quat(Q_QX) + quat(Q_QY)*quat(Q_QY));

  phi = atan( dcm12, dcm22 );
  theta = -asin( dcm02 );
  psi = atan( dcm01, dcm00 );

  euler = [phi; theta; psi];
endfunction


//
//
//
function [qo] = quat_normalize(qi)
  qo = qi / norm(qi);
endfunction

//
//
//
function [vo] = quat_vect_mult(q, vi)
  dcm = dcm_of_quat(q);
  vo = dcm * vi;
endfunction

//
//
//
function [vo] = quat_vect_inv_mult(q, vi)
  dcm = dcm_of_quat(q);
  vo = dcm' * vi;
endfunction


//
//
//  q_a2c = q_a2b comp q_b2c , aka  q_a2c = q_b2c * q_a2b
//
function [q_a2c] = quat_comp(q_a2b, q_b2c)
    M = [ q_b2c(Q_QI) -q_b2c(Q_QX) -q_b2c(Q_QY) -q_b2c(Q_QZ)
	  q_b2c(Q_QX)  q_b2c(Q_QI)  q_b2c(Q_QZ) -q_b2c(Q_QY)
	  q_b2c(Q_QY) -q_b2c(Q_QZ)  q_b2c(Q_QI)  q_b2c(Q_QX)
	  q_b2c(Q_QZ)  q_b2c(Q_QY) -q_b2c(Q_QX)  q_b2c(Q_QI) ];
    q_a2c = M * q_a2b;
endfunction

//
//
// q_a2b = q_a2b comp_inv q_b2c , aka  q_a2b = qinv(_b2c) * q_a2c
//
function [q_a2b] = quat_comp_inv(q_a2c, q_b2c)
  q_c2b = quat_inv(q_b2c);
  q_a2b = quat_comp(q_a2c, q_c2b);
endfunction

//
//
// q_a2c = q_b2a inv_comp q_b2c , aka  q_a2b = qinv(_b2c) * q_a2c
//
function [q_a2c] = quat_inv_comp(q_b2a, q_b2c)
  q_a2b = quat_inv(q_b2a);
  q_a2c = quat_comp(q_a2b, q_b2c);
endfunction


function [b] = quat_div(a, c)
    M = [ c(Q_QI) +c(Q_QX) +c(Q_QY) +c(Q_QZ)
	  c(Q_QX) -c(Q_QI) -c(Q_QZ) +c(Q_QY)
	  c(Q_QY) +c(Q_QZ) -c(Q_QI) -c(Q_QX)
	  c(Q_QZ) -c(Q_QY) +c(Q_QX) -c(Q_QI) ];
    b = M * a;
endfunction



//
//
//
function [b] = quat_inv(a)
  b(Q_QI) =  a(Q_QI);
  b(Q_QX) = -a(Q_QX);
  b(Q_QY) = -a(Q_QY);
  b(Q_QZ) = -a(Q_QZ);
endfunction


//
//
//
function [b] = quat_wrap_shortest(a)
  if a(Q_QI) < 0
    b = quat_explementary(a);
  else
    b = a;
  end
endfunction


//
//
//
function [b] = quat_explementary(a)
  b(Q_QI) = -a(Q_QI);
  b(Q_QX) = -a(Q_QX);
  b(Q_QY) = -a(Q_QY);
  b(Q_QZ) = -a(Q_QZ);
endfunction

//
//
//
function [b] = quat_wrap_shortest(a)
  if a(Q_QI) < 0
    b = quat_explementary(a);
  else
    b = a;
  end
endfunction

//
//
//
function [omega] = get_omega_quat(rate)
  p = rate(1);
  q = rate(2);
  r = rate(3);
  omega = [ 0   p   q   r
           -p   0  -r   q
           -q   r   0  -p
           -r  -q   p   0 ];
endfunction

//
//
//
function [ c ] = cross_product(a, b)
  c = [ a(2)*b(3) - a(3)*b(2)
        a(3)*b(1) - a(1)*b(3)
        a(1)*b(2) - a(2)*b(1) ];
endfunction
