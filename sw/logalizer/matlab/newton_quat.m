function [est_quat, norm_err, nb_iter] = newton_quat(start_quat, prec, max_iter, ...
						     accel_ned, mag_ned, ...
						     accel_body, mag_body)

ref = [accel_ned 
       mag_ned];

meas = [accel_body 
	mag_body];

cur_quat = start_quat;
iter = 0;

while (1)

  R = dcm_of_quat(cur_quat)';

  M = [ R        zeros(3)
	zeros(3) R        ];


  err = [accel_ned ; mag_ned] - M * [accel_body ; mag_body];

  norm_err = norm(err);
  if (norm_err < prec || iter > max_iter)
    est_quat = cur_quat; 
    nb_iter = iter;
    return;
  end;

  q0 = cur_quat(1);
  q1 = cur_quat(2);
  q2 = cur_quat(3);
  q3 = cur_quat(4);

  dR_dq0 = 2 * [ q0 -q3  q2
		 q3  q0 -q1 
		 -q2  q1  q0 ];

  dR_dq1 = 2 * [  q1  q2  q3
		  q2 -q1 -q0
		  q3  q0 -q1 ]; 

  dR_dq2 = 2 * [ -q2  q1  q0
		 q1  q2  q3
		 -q0  q3 -q2 ]; 

  dR_dq3 = 2 * [ -q3 -q0  q1
		 q0 -q3  q2
		 q1  q2  q3 ];

  dM_dq0 = [ dR_dq0   zeros(3)
	     zeros(3) dR_dq0  ];

  dM_dq1 = [ dR_dq1   zeros(3)
	     zeros(3) dR_dq1  ];

  dM_dq2 = [ dR_dq2   zeros(3)
	     zeros(3) dR_dq2  ];

  dM_dq3 = [ dR_dq3   zeros(3)
	     zeros(3) dR_dq3  ];

  J = [ dM_dq0*meas dM_dq1*meas dM_dq2*meas dM_dq3*meas ];

  %  lp = J' * J;
  %  rp = -J' * err;

  delta_quat = inv(J' * J) * J' * err;
  cur_quat = normalize_quat(cur_quat + delta_quat);
  iter = iter + 1;

end;


