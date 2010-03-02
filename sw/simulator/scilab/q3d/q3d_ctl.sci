CTL_UT    = 1;
CTL_UD    = 2;
CTL_USIZE = 2;

global ctl_u;

//           X    Z  Theta   Xd    Zd   Theta_d
//ctl_gain = [ 0   -1    0      0    -1    0
//            -2    0    5     -2     0    5 ];

if 0
ctl_gain = [ 0    0    0      0     0    0
             0    0    0      0     0    0 ];
else
ctl_gain = [    0   -1    0      0       -1    0
             0.95    0   -3      1.2      0   -3 ];
	
end

function ctl_init(time)

  global ctl_u;
  ctl_u = zeros(CTL_USIZE, length(time));
  
endfunction

function [fb_cmd] = ctl_compute_feeback(fdm_state, ref)

  state_err = fdm_state - ref;
  fb_cmd = ctl_gain * state_err;

endfunction




