
//
//
//
// 
//
//
//
function [X1, P1] = ekf_predict_continuous(X0, X0dot, dt, P0, F, Q)

X1 = X0 + X0dot * dt;
Pdot = F*P + P*F' + Q;
P = P + Pdot * dt;

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
Pdot = F*P*F' + Q;
P1 = P0 + Pdot * dt;

endfunction

//
//
//
// 
//
//
//
function [X1, P1] = ekf_update(X0, P0, H, R, err)

     E = H * P * H' + R;
     K = P * H' * inv(E);
     P = P - K * H * P;
     X = X + K * err;
  
endfunction