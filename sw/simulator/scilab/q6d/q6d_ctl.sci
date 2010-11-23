CTL_UT    = 1;
CTL_UP    = 2;
CTL_UQ    = 3;
CTL_UR    = 4;
CTL_USIZE = 4;


global ctl_diff_flat_cmd;
global ctl_diff_flat_ref;
global ctl_fb_cmd;
global ctl_u;
global ctl_motor_cmd;

function ctl_init(time)

  global ctl_diff_flat_cmd;
  ctl_diff_flat_cmd = zeros(CTL_USIZE,length(time));
  global ctl_diff_flat_ref;
  ctl_diff_flat_ref = zeros(DF_REF_SIZE, length(time));
  global ctl_fb_cmd;
  ctl_fb_cmd = zeros(CTL_USIZE,length(time));
  global ctl_u;
  ctl_u = zeros(CTL_USIZE, length(time));
  global ctl_motor_cmd;
  ctl_motor_cmd = zeros(CTL_USIZE,length(time));

endfunction



function ctl_run(i, fo_tra)

  global ctl_diff_flat_cmd;
  ctl_diff_flat_cmd(:,i) = df_input_of_fo(fo_tra);
  global ctl_diff_flat_ref;
  ctl_diff_flat_ref(:,i) = df_ref_of_fo(fo_tra);
  global ctl_u;
  ctl_u(:,i) = ctl_diff_flat_cmd(:,i);
  motor_of_cmd = [ 0.25    0.     0.5   -0.25
                   0.25   -0.5    0.     0.25
                   0.25    0.    -0.5   -0.25
                   0.25    0.5    0.     0.25 ];
  global ctl_motor_cmd;
  ctl_motor_cmd(:,i) =  1/fdm_Ct0 * motor_of_cmd * ctl_u(:,i);

endfunction
