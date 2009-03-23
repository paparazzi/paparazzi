clear();
clearglobal();


exec('q3d_utils.sci');
exec('q3d_fdm.sci');
exec('q3d_ctl.sci');
exec('q3d_ref_misc.sci');
exec('poly_utils.sci');


start = [ -10; 0
           0; 0
	   0; 0
	   0; 0
	   0; 0 ];

circle_center =  [  0; 0 ];
       
       
stop  = [  200; 0
           0; 0
	   0; 0
	   0; 0
	   0; 0 ];

if 0
  time_ref = [0];
  ref = start;
  
  // stay
  //[time_ref, ref] = get_reference_poly3(time_ref, ref, 1, start);
  [time_ref, ref] = get_reference_poly3(time_ref, ref, 20, stop);
  //[time_ref, ref] = get_reference_poly3(time_ref, ref, 1, stop);
  
  
  clf();
  ref_display(time_ref, ref);
  pause
end

time_ref = [0];
ref = start;
[time_ref, ref] = get_reference_lti4(time_ref, ref, 10, stop(1:2));

clf();
ref_display(time_ref, ref);
pause

fdm_init(time_ref, ref);
global fdm_state

ctl_init();

global ctl_motor;
ctl_motor(:,1) = fdm_mass * fdm_g * [0.5;0.5];

global ctl_ref_0;
global ctl_ref_1;
global ctl_ref_2;
global ctl_ref_3;
global ctl_ref_4;
ctl_ref_0 =  ref(1:2,:);
ctl_ref_1 =  ref(3:4,:);
ctl_ref_2 =  ref(5:6,:);
ctl_ref_3 =  ref(7:8,:);
ctl_ref_4 =  ref(9:10,:);


for i=1:length(fdm_time)-1

  fdm_run(i+1, ctl_motor(:,i));
  ctl_run_flatness(i+1);
  
end
if 1
clf();
plot2d(fdm_state(FDM_SX,:), fdm_state(FDM_SZ,:),2);
plot2d(ctl_ref_0(AXIS_X,:), ctl_ref_0(AXIS_Z,:),3);
  pause
end
clf();
ctl_display();
  pause

gen_video();