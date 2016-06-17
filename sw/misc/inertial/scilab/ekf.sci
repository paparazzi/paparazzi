
//
//
//
// 
//
//
//
function [X1, P1] = ekf_predict_continuous(X0, X0dot, dt, P0, F, Q)

X1 = X0 + X0dot * dt;
P0dot = F*P0 + P0*F' + Q;
P1 = P0 + P0dot * dt;

endfunction

//
//
//
// 
//
//
//
function [X1, P1] = ekf_predict_discrete(X0, X0dot, dt, P0, F, Q)

X1 = X0 + X0dot * dt;
expF = expm(dt * F);
P0dot = expF*P0*expF' + Q;
P1 = P0 + P0dot * dt;

endfunction

//
//
//
// 
//
//
//
function [X1, P1] = ekf_update(X0, P0, H, R, err)

E = H * P0 * H' + R;
K = P0 * H' * inv(E);
P1 = P0 - K * H * P0;
X1 = X0 + K * err;

endfunction



//
// Pade approximation of matrix exponential
//
function [expA] = mat_exp(A, epsilon)

  normA = norm(A, 'inf');
  //Ns = max(0, int(normA)) ???
  Ns = normA;
  ki = 2^(-Ns) * A;

  i=1;
//  foo = 2^(3-2*i)*


expA = A;


endfunction


