

function [X0, P0] = tilt_init(avg_len, accel, gyro)

  avg_accel = sum(accel(:,1:avg_len), 'c') / avg_len;
  avg_gyro = sum(gyro(:,1:avg_len), 'c')   / avg_len;

  X0 = [ phi_of_accel(avg_accel)
         avg_gyro(1) ];
  
  P0 = [ 1 0
         0 1 ];


endfunction
   



function [] = tilt_display(time, X, P)

xbasc();
subplot(2,1,1)
xtitle('Angle');
plot2d([time]', X(1,:)', style=[5]);

subplot(2,1,2)
xtitle('Bias');
plot2d([time]', X(2,:)', style=[5]);


endfunction
   