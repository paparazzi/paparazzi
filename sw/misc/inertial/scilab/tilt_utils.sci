

function [X0, P0] = tilt_init(avg_len, accel, gyro)

  avg_accel = sum(accel(:,1:avg_len), 'c') / avg_len;
  avg_gyro = sum(gyro(:,1:avg_len), 'c')   / avg_len;

  X0 = [ phi_of_accel(avg_accel)
         avg_gyro(1) ];
  
  P0 = [ 1 0
         0 1 ];

endfunction
   

function [Pi] = tilt_get_P(P, i)
  
  Pi = P(:, 1+2*(i-1):2+2*(i-1));

endfunction



function [] = tilt_display(time, X, P, M)

xbasc();
subplot(3,1,1)
xtitle('Angle');
EstAngleDeg = X(1,:) * 180 / %pi;
MeasAngleDeg = M * 180 / %pi;
plot2d([time; time]', [EstAngleDeg; MeasAngleDeg]', style=[5, 3], leg="estimated@measure");

subplot(3,1,2)
xtitle('Bias');
EstBiasDeg = X(2,:) * 180 / %pi;
plot2d([time]', EstBiasDeg', style=[5]);


subplot(3,1,3)
xtitle('Covariance');
P11 = [];
P22 = [];
for i=1:length(time)
  P11 = [P11 P(1, 1+2*(i-1))];
  P22 = [P22 P(2, 2+2*(i-1))];
end
plot2d([time; time]', [P11; P22]', style=[5 3], leg="angle@bias");

endfunction
   