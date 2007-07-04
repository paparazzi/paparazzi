
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
P0dot = F*P0*F' + Q;
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