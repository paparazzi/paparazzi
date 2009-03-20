clear();
clearglobal();


exec('q3d_utils.sci');
exec('q3d_fdm.sci');
exec('q3d_ctl.sci');
exec('q3d_ref_misc.sci');
exec('poly_utils.sci');

global fdm_state

[time_loop, ref_loop] = get_reference_circle();

a = [-1 0 0 0 0;
      0 0 0 0 0];

[time_foo, ref_foo] = get_reference_poly3(1,a ,a);
  
  
start_loop = [ref_loop(1,1) ref_loop(3,1) ref_loop(5,1) ref_loop(7,1) ref_loop(9,1)
              ref_loop(2,1) ref_loop(4,1) ref_loop(6,1) ref_loop(8,1) ref_loop(10,1)];

[time_intro, ref_intro] = get_reference_poly3(0.9, a, start_loop);

end_loop = [ref_loop(1,$),ref_loop(3,$),ref_loop(5,$),ref_loop(7,$),ref_loop(9,$);
            ref_loop(2,$),ref_loop(4,$),ref_loop(6,$),ref_loop(8,$),ref_loop(10,$)];

c = [ 1 0 0 0 0;
      0 0 0 0 0];

[time_outro, ref_outro] = get_reference_poly3(0.9,end_loop,c);

[time_bar, ref_bar] = get_reference_poly3(1,c ,c);


Xref = [ref_foo ref_intro(:,2:$)  ref_loop(:,2:$)  ref_outro(:,2:$) ref_bar(:,2:$)];


fdm_init(0,time_foo($)+time_intro($)+time_loop($)+time_outro($)+time_bar($));
fdm_state(FDM_SX,1) = -1;

ctl_init();

global ctl_motor;
ctl_motor(:,1) = fdm_mass * fdm_g * [0.5;0.5];

global ctl_ref_0;
global ctl_ref_1;
global ctl_ref_2;
global ctl_ref_3;
global ctl_ref_4;
ctl_ref_0 =  Xref(1:2,:);
ctl_ref_1 =  Xref(3:4,:);
ctl_ref_2 =  Xref(5:6,:);
ctl_ref_3 =  Xref(7:8,:);
ctl_ref_4 =  Xref(9:10,:);

//global fdm_state;
fdm_state(FDM_SXD, 1) = ctl_ref_1(AXIS_X,1);
fdm_state(FDM_SZD, 1) = ctl_ref_1(AXIS_Z,1);
ctl_run_flatness(1);
//global ctl_ref_thetad;
fdm_state(FDM_STHETA,  1) = ctl_ref_theta(1);
fdm_state(FDM_STHETAD, 1) = ctl_ref_thetad(1);

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