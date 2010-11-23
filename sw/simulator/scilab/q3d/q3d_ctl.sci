CTL_UT    = 1;
CTL_UD    = 2;
CTL_USIZE = 2;

global ctl_diff_flat_cmd;
global ctl_diff_flat_ref;
global ctl_fb_cmd;
global ctl_u;
global ctl_motor_cmd;

fb_o_x = rad_of_deg(250);
fb_x_x = 0.9;

fb_o_z = rad_of_deg(250);
fb_x_z = 0.9;

fb_o_t = rad_of_deg(1050);
fb_x_t = 0.9;

if 0
ctl_gain = [ 0    0    0      0     0    0
             0    0    0      0     0    0 ];
else
ctl_gain = [    0   -1    0      0       -1    0
             0.95    0   -3      1.2      0   -3 ];

end

function ctl_init(time)
  global ctl_diff_flat_cmd;
  ctl_diff_flat_cmd = zeros(CTL_USIZE,length(time));
  global ctl_diff_flat_ref;
  ctl_diff_flat_ref = zeros(FDM_SSIZE, length(time));
  global ctl_fb_cmd;
  ctl_fb_cmd = zeros(CTL_USIZE,length(time));
  global ctl_u;
  ctl_u = zeros(CTL_USIZE, length(time));
  global ctl_motor_cmd;
  ctl_motor_cmd = zeros(CTL_USIZE,length(time));
endfunction

function ctl_run(i, model_a, model_b)

  global ctl_diff_flat_cmd;
  ctl_diff_flat_cmd(:,i) = df_input_of_fo(fo_traj(:,:,i), model_a, model_b);
  global ctl_diff_flat_ref;
  ctl_diff_flat_ref(:,i) = df_state_of_fo(fo_traj(:,:,i));
  global ctl_fb_cmd;
  ctl_fb_cmd(:,i) = ctl_compute_feeback(fdm_state(:,i), ctl_diff_flat_ref(:,i), ctl_diff_flat_cmd(:,i), model_a, model_b);
  global ctl_u;
  ctl_u(:,i) = ctl_diff_flat_cmd(:,i) + ctl_fb_cmd(:,i);
  MotorsOfCmds = 0.5*[1 -1 ; 1 1];
  global ctl_motor_cmd;
  ctl_motor_cmd(:,i) =  MotorsOfCmds * ctl_u(:,i);

endfunction



function [fb_cmd] = ctl_compute_feeback(fdm_state, s_ref, u_ref, a, b)

  state_err = fdm_state - s_ref;

  st = sin(s_ref(FDM_STHETA));
  ct = cos(s_ref(FDM_STHETA));
  o2_x = fb_o_x^2;
  xo2_x = 2*fb_x_x*fb_o_x;
  o2_z = fb_o_z^2;
  xo2_z = 2*fb_x_z*fb_o_z;
  o2_t = fb_o_t^2;
  xo2_t = 2*fb_x_t*fb_o_t;
  ut = u_ref(1);


  gain = [ o2_x*st/a            -o2_z*ct/a                0      xo2_x*st/a             -xo2_z*ct/a                   0
           o2_t/b*o2_x*ct/a/ut   o2_t/b*o2_z*st/a/ut   -o2_t/b   o2_t/b*xo2_x*ct/a/ut   o2_t/b*xo2_z*st/a/ut    -xo2_t/b];

  fb_cmd = gain * state_err;

endfunction




