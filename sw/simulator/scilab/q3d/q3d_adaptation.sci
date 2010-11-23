//
//
//  Model parameters estimation
//
//

// estimated parameters
ADP_EST_A    = 1;  // a = fdm_Ct0/fdm_mass
ADP_EST_B    = 2;  // b = fdm_la*fdm_Ct0/fdm_inertia
ADP_EST_SIZE = 2;

global adp_est;
global adp_P;
global adp_y;

adp_dt = 1/512;
adp_lp_tau = 0.75;
adp_lp_alpha = adp_dt/(adp_dt+adp_lp_tau);

global adp_thetad_f;
global adp_ud_f;


function adp_init(time, est0, P0)

  global adp_est;
  adp_est = zeros(ADP_EST_SIZE, length(time));
  adp_est(:,1) = est0;

  global adp_P;
  adp_P = zeros(ADP_EST_SIZE, ADP_EST_SIZE, length(time));
  adp_P(:,:,1) = [ 20 0 ; 0 50000];

  global adp_y;
  adp_y = zeros(ADP_EST_SIZE, length(time));

  global adp_thetad_f;
  adp_thetad_f = zeros(1, length(time));
  global adp_ud_f;
  adp_ud_f = zeros(1, length(time));


endfunction


// propagate the adaptation from step i-1 to step 1
function adp_run2(i)

  // low pass filter thetad and ud
  global adp_thetad_f;
  adp_thetad_f(i) = adp_lp_tau*adp_lp_alpha*sensors_state(SEN_SG,i) + (1-adp_lp_alpha)*adp_thetad_f(i-1);
  global adp_ud_f;
  adp_ud_f(i)     = adp_lp_tau*adp_lp_alpha*ctl_u(CTL_UD,i)         + (1-adp_lp_alpha)*adp_ud_f(i-1);

  // output
  global adp_y;
  adp_y(:,i) = [ sqrt(fdm_accel(FDM_AX,i)^2 + (fdm_accel(FDM_AZ,i)+9.81)^2)      // FIXME
                 sensors_state(SEN_SG,i) - 1/adp_lp_tau * adp_thetad_f(i)    ];
  // input
  W = [ ctl_u(CTL_UT,i) 0; 0 adp_ud_f(i) ];

  global adp_est;
  global adp_P;
  // residual
  e1 = W*adp_est(:,i-1) - adp_y(:,i);
  // update state
  ad = -adp_P(:,:,i-1)*W'*e1;
  // first order integration
  adp_est(:,i) = adp_est(:,i-1) + ad * adp_dt;
//  pause

  // update gain
//  lambda = 0.25*(1-norm(adp_P(:,:,i-1))/2.5);
    lambda = 1.025;
  Pd = lambda*adp_P(:,:,i-1) - adp_P(:,:,i-1)* W' * W * adp_P(:,:,i-1);
  adp_P(:,:,i) = adp_P(:,:,i-1) + Pd * dt;

endfunction

// propagate the adaptation from step i-1 to step 1
function adp_run(i)
  // low pass filter thetad and ud
  global adp_thetad_f;
  adp_thetad_f(i) = adp_lp_tau*adp_lp_alpha*sensors_state(SEN_SG,i-1) + (1-adp_lp_alpha)*adp_thetad_f(i-1);
  global adp_ud_f;
  adp_ud_f(i)     = adp_lp_tau*adp_lp_alpha*ctl_u(CTL_UD,i-1)         + (1-adp_lp_alpha)*adp_ud_f(i-1);

  global adp_y;
  global adp_est;
  global adp_P;

  //output
  adp_y(1,i-1) = sqrt(fdm_accel(FDM_AX,i-1)^2 + (fdm_accel(FDM_AZ,i-1)+9.81)^2);   // fixme
  // input
  W = ctl_u(CTL_UT,i-1);
  // residual
  e1 = W*adp_est(1,i-1) - adp_y(1,i-1);
  // update state
  ad = -adp_P(1,1,i-1)*W'*e1;
  adp_est(1,i) = adp_est(1,i-1) + ad * adp_dt;
  // update gain
  lambda = 2.5*(1-abs(adp_P(1,1,i-1))/20);
//  lambda = 2.25;
  Pd = lambda*adp_P(1,1,i-1) - adp_P(1,1,i-1)* W' * W * adp_P(1,1,i-1);
  adp_P(1,1,i) = adp_P(1,1,i-1) + Pd * dt;

  //output
  adp_y(2,i-1) = sensors_state(SEN_SG,i-1) - 1/adp_lp_tau * adp_thetad_f(i-1);
  // input
  W =  adp_ud_f(i-1);
  // residual
  e1 = W*adp_est(2,i-1) - adp_y(2,i-1);
  // update state
  ad = -adp_P(2,2,i-1)*W'*e1;
  adp_est(2,i) = adp_est(2,i-1) + ad * adp_dt;
  // update gain
  lambda = 2.5*(1-abs(adp_P(2,2,i-1))/50000);
//  lambda = 2.25;
  Pd = lambda*adp_P(2,2,i-1) - adp_P(2,2,i-1)* W' * W * adp_P(2,2,i-1);
  adp_P(2,2,i) = adp_P(2,2,i-1) + Pd * dt;

endfunction

