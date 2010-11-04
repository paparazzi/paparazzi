%% EULERS OF QUATERNION
%
% [euler] = eulers_of_quat(quat)
%
% transposes a quaternion to euler angles
function [euler] = eulers_of_quat(quat)
  
   algebra_common;

  if size(quat)(2)==4
     quat = quat';
     transpose = 1;
  end

  dcm00 = 1.0 - 2*(quat(Q_QY,:).*quat(Q_QY,:) + quat(Q_QZ,:).*quat(Q_QZ,:));
   dcm01 =       2*(quat(Q_QX,:).*quat(Q_QY,:) + quat(Q_QI,:).*quat(Q_QZ,:));
   dcm02 =       2*(quat(Q_QX,:).*quat(Q_QZ,:) - quat(Q_QI,:).*quat(Q_QY,:));
  dcm12 =       2*(quat(Q_QY,:).*quat(Q_QZ,:) + quat(Q_QI,:).*quat(Q_QX,:));
  dcm22 = 1.0 - 2*(quat(Q_QX,:).*quat(Q_QX,:) + quat(Q_QY,:).*quat(Q_QY,:));

  phi = atan2( dcm12, dcm22 );
  theta = -asin( dcm02 );
  psi = atan2( dcm01, dcm00 );

  euler = [phi; theta; psi];
 if transpose
  euler = euler';
 end
endfunction
