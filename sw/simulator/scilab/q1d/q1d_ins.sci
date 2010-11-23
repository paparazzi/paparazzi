


INS_Z     = 1;
INS_ZD    = 2;
INS_BIAS  = 3;
INS_SIZE  = 3;
INS_ZDD   = 4;  // WARNING : hack - byproduct
                // keeping size to 3 for covariance

function [Xi1, Pi1] = ins_run(Xi, Pi, sensors_i, sensors_i1, dt)

  //
  // propagate
  //
  F = [ 1 dt -dt^2/2
        0  1 -dt
        0  0   1     ];
  B = [ dt^2/2 dt 0]';

  Qz  = 0.01*dt^2/2;
  Qzd = 0.01*dt;

  // FIXME: Qz and Qzd noise mismatch with dt
  //Qz  = 0.01*dt;
  //Qzd = 0.01*dt^2/2;


  Qbias = 0.0001 * dt;
  Q = [ Qz  0    0
         0  Qzd  0
         0  0    Qbias ];

  accel = sensors_i( SENSORS_ACCEL ) - 9.81;

  Xi1m = F * Xi + B * accel;

  Pi1m = F * Pi * F' + Q;

  //
  // Update
  //
  H = [1 0 0];
  R = 0.1;
  // state residual
  y = sensors_i1( SENSORS_BARO ) - H * Xi1m;
  // covariance residual
  S = H*Pi1m*H' + R;
  // kalman gain
  K = Pi1m*H'*inv(S);
  // update state
  Xi1 = Xi1m + K*y;
  // update covariance
  Pi1 = Pi1m - K*H*Pi1m;
  Xi1(4) = accel - Xi1(INS_BIAS);

endfunction

function [Pi] = getP(n, P,i)
  Pi = P(:,(i-1)*n+1:i*n);
endfunction


//
// Simple display
//
function ins_display_simple(Xins, Pins, Xfdm, Xsensors, time)

nr = 3;
nc = 2;

subplot(nr,nc,1);
plot2d(time, Xsensors(SENSORS_BARO,:),3);
plot2d(time, Xfdm(FDM_Z,:),2);
plot2d(time, Xins(INS_Z,:), 5);
legends(["Estimation", "Truth", "Measurement"],[5 2 3], with_box=%f, opt="ur");
xtitle('Altitude');

subplot(nr,nc,3);
plot2d(time, Xfdm(FDM_ZD,:), 2);
plot2d(time, Xins(INS_ZD,:), 5);
legends(["Estimation", "Truth"],[5 2], with_box=%f, opt="ur");
xtitle('Vertical Speed');

subplot(nr,nc,5);
plot2d(time, Xfdm(FDM_ZDD,:),2);
plot2d(time, Xins(4,:), 5);
legends(["Estimation", "Truth"],[5 2], with_box=%f, opt="ur");
xtitle('Vertical Acceleration');

subplot(nr,nc,2);
plot2d(time, 1.0*Xsensors(SENSORS_ACCEL_BIAS,:),2);
plot2d(time, Xins(INS_BIAS,:), 5);
legends(["Estimation", "Truth"],[5 2], with_box=%f, opt="ur");
xtitle('Accelerometer Bias');

subplot(nr,nc,4);
Pzz = zeros(1,length(time));
Pzdzd = zeros(1,length(time));
for i=1:length(time)
  Pi = getP(INS_SIZE, Pins, i);
  Pzz(i) = Pi(INS_Z, INS_Z);
  Pzdzd(i) = Pi(INS_ZD, INS_ZD);
  Pbb(i) = Pi(INS_BIAS, INS_BIAS);
end

plot2d(time, Pzz, 5);
plot2d(time, Pzdzd, 2);
plot2d(time, Pbb, 3);
legends(["Altitude", "Speed", "Bias"],[5 2 3], with_box=%f, opt="ur");
xtitle('Estimation Covariance');



endfunction

