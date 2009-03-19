clear();
clearglobal();


exec('q3d_utils.sci');
exec('q3d_fdm.sci');
exec('q3d_ctl.sci');
exec('q3d_ref_misc.sci');
exec('poly_utils.sci');

global fdm_state

a = [-1 0 0 0 0;
    0 0 0 0 0];

b = [0 1.2566371 0 -1.9844017 0;
    0 0 1.5791367 0 -2.4936727];

[time1, Xref1] = get_reference_poly3(5, a, b);
[time2, Xref2] = get_reference_circle();

c = [Xref2(1,$),Xref2(3,$),Xref2(5,$),Xref2(7,$),Xref2(9,$);
    Xref2(2,$),Xref2(4,$),Xref2(6,$),Xref2(8,$),Xref2(10,$)];
a(1,1) = 3;
a(2,1) = 1;

[time3, Xref3] = get_reference_poly3(4,c,a);

Xref = [Xref1  Xref2(:,2:$)  Xref3(:,2:$)];


fdm_init(0,time1($)+time2($)+time3($));
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