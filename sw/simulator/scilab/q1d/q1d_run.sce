clear();

exec("q1d_utils.sci");
exec("q1d_fdm.sci");
exec("q1d_sensors.sci");
exec("q1d_ins.sci");
exec("q1d_ctl_common.sci");
exec("q1d_ctl_miac.sci");
exec("q1d_ctl_mrac.sci");


//
// Duration of the simulation
//
ts = 0;
te = 14;
dt = 1./512.;
time = ts:dt:te;

//
// type of control
//
CTL_MIAC = 0;
CTL_MRAC = 1;
ctl_type = CTL_MIAC;


//
// input trajectory
//
// single input - height
ctl_sp = zeros(1, length(time));
if 1
  // step 1m 5s
  k = find ( time > 3. & time < 6.);
  ctl_sp(k) = 1.;
end
if 0
  // step 15m 14s
  k = find ( time > 1. & time < 7);
  ctl_sp(k) = 15.;
end
if 0
  // 1rad/s sine input
  omega = 2.;
  ctl_sp = 3.5 * sin(omega * time);
end


//
// Flight dynamic model
// aka true state
//
fdm_state   = zeros(FDM_SIZE, length(time));
fdm_perturb = zeros(1, length(time));
fdm_param = fdm_mass * ones(1, length(time));
fdm_state(:,1) = [ 0; 0; 0];
if 0
  k = find(time >= 5 & time < 5.25);
  fdm_perturb(k) = 20.;
end
if 1
  k = find(time >= 5);
  fdm_param(k) = 0.75*fdm_mass;
end


//
// Sensor Model state
//
sensors_state = zeros(SENSORS_SIZE, length(time));

//
// State Estimation ( INS )
//
BYPASS_INS = 0;
ins_state = zeros(INS_SIZE+1, length(time));
ins_state(:,1) = [ 0; 0; 0; 0];
ins_cov = zeros(INS_SIZE, INS_SIZE * length(time));
ins_cov(1:INS_SIZE,1:INS_SIZE) = [ 1 0 0
                                   0 1 0
				   0 0 1 ];


//
// reference model
//
ctl_ref_state = zeros(REF_SIZE, length(time));
// parameter of the inverted model
ctl_adp_state = zeros(1, length(time));
ctl_adp_state(1) = 35;
ctl_adp_cov   = zeros(1, length(time));
ctl_adp_cov(1) = 0.05;
ctl_adp_meas  = zeros(1, length(time));

// output of the controller
ctl_command   = zeros(1, length(time));
ctl_command(1) = 1;
ctl_feed_fwd = zeros(1, length(time));
ctl_feed_fwd(1) = 1;
ctl_max = zeros(1, length(time));
ctl_max(1) = 0;
ctl_min = zeros(1, length(time));
ctl_min(1) = 0;

for i = 1:length(time)-1

  ti  = time(i);
  ti1 = time(i+1);
  // run control
  Xinsi = ins_state(:,i);
  if i==1
    im1 = i;
  else
    im1 = i - 1;
  end
  Xspi = ctl_sp(:,i);
  Xrefim1 = ctl_ref_state(:,im1);
  Xadpim1 = ctl_adp_state(:,im1);
  Padpim1 = ctl_adp_cov(:,im1);
  Uim1 = ctl_command(:,im1);
  select ctl_type
  case CTL_MIAC
    [Xrefi, Ui, Xadpi, Padpi, Madpi, feed_fwd] = ctl_miac_run(Xinsi, Xrefim1, Xspi, dt, Xadpim1, Padpim1, Uim1);
  case CTL_MRAC
    [Xrefi, Ui, Xadpi, feed_fwd] = ctl_mrac_run(Xinsi, Xrefim1, Xspi, dt, Xadpim1, Uim1);
  end
  ctl_ref_state(:,i) = Xrefi;
  ctl_command(:,i) = Ui;
  ctl_feed_fwd(:,i) = feed_fwd;
  ctl_max(:,i) = max([Ui feed_fwd]);
  ctl_min(:,i) = min([Ui feed_fwd]);
  ctl_adp_state(:,i) = Xadpi;
  if ctl_type == CTL_MIAC
    ctl_adp_cov(:,i) = Padpi;
    ctl_adp_meas(:,i) = Madpi;
  end
  // run_fdm
  Xfdmi  = fdm_state(:,i);
  Xfdmi1 = fdm_run(Xfdmi, Ui, ti, ti1, fdm_perturb(:,i), fdm_param(i));
  fdm_state(:,i+1) = Xfdmi1;
  // run sensor model
  Xsensorsi1 = sensors_run(ti, Xfdmi);
  sensors_state(:,i+1) = Xsensorsi1;
  // run ins
  Pinsi = getP(INS_SIZE, ins_cov, i);
  Xsensorsi = sensors_state(:,i);
  [Xinsi1, Pinsi1] = ins_run(Xinsi(1:INS_SIZE), Pinsi, Xsensorsi, Xsensorsi1, dt);
  ins_state(:,i+1) = Xinsi1;
  ins_cov(:,(i)*INS_SIZE+1:(i+1)*INS_SIZE) = Pinsi1;
  //
  if BYPASS_INS
    ins_state(INS_Z,i+1) = fdm_state(FDM_Z,i+1);
    ins_state(INS_ZD,i+1) = fdm_state(FDM_ZD,i+1);
    ins_state(INS_ZDD,i+1) = fdm_state(FDM_ZDD,i+1);
  end
end

ctl_max = [0 ctl_max 0];
ctl_min = [0 ctl_min 0];
time_exp = [0 time te];
ctl_list = list(ctl_command, ctl_feed_fwd, ctl_max, ctl_min, time_exp);

if 0

  set("current_figure",0);
  clf();
  f=get("current_figure");
  f.figure_name="FDM";
  drawlater();
  fdm_display_simple(fdm_state, time);
  drawnow();

  set("current_figure",1);
  clf();
  f=get("current_figure");
  f.figure_name="Sensors";
  drawlater();
  sensors_display_simple(sensors_state, time);
  drawnow();

end

if 1

  set("current_figure",2);
  clf();
  f=get("current_figure");
  f.figure_name="INS";
  drawlater();
  ins_display_simple(ins_state, ins_cov, fdm_state, sensors_state, time);
  drawnow();


set("current_figure",3);
clf();
f=get("current_figure");
f.figure_name="Control";
drawlater();
ctl_display(ctl_type, ctl_list, ctl_sp, ctl_ref_state, ins_state, ctl_adp_state, ctl_adp_cov, ctl_adp_state, time);
drawnow();

end

if 0

set("current_figure",4);
clf();
f=get("current_figure");
f.figure_name="Control - Reference";
drawlater();
ctl_display_ref(ctl_sp, ctl_ref_state, time);
drawnow();

end


//
// big filter play
//
exec("q1d_big_filter.sci");
bfl_init(time);
for i = 2:length(time)
   bfl_predict(i,  ctl_command(i-1), dt);
   if modulo(i,5) == 0
     bfl_update_baro(i, sensors_state(SENSORS_BARO,i));
   end
   bfl_update_accel(i, sensors_state(SENSORS_ACCEL,i), ctl_command(i-1));
end

set("current_figure",5);
clf();
f=get("current_figure");
f.figure_name="Big Filter";
drawlater();

  subplot(4, 2, 1);
  plot2d(time, sensors_state(SENSORS_BARO,:),3);
  plot2d(time, bflt_state(BF_Z, :), 5);
  plot2d(time, fdm_state(FDM_Z,:),2);
  legends(["Estimation", "Truth", "Measurement"],[5 2 3], with_box=%f, opt="ur");
  xtitle('Z');

  subplot(4, 2, 3);
  plot2d(time, bflt_state(BF_ZD, :),5);
  plot2d(time, fdm_state(FDM_ZD,:),2);
  legends(["Estimation", "Truth"],[5 2], with_box=%f, opt="ur");
  xtitle('ZD');

  subplot(4, 2, 5);
  plot2d(time, sensors_state(SENSORS_ACCEL_BIAS,:),2);
  plot2d(time, bflt_state(BF_BIAS, :), 5);
  legends(["Estimation", "Truth"],[5 2], with_box=%f, opt="ur");
  xtitle('BIAS');

  subplot(4, 2, 7);
  plot2d(time, bflt_state(BF_C, :));
  xtitle('C');


  subplot(4, 2, 2);
  foo=[];
  for i=1:length(time)
    foo = [foo bflt_cov(BF_Z, BF_Z, i)];
  end
  plot2d(time, foo);
  xtitle('COV ZZ');


  subplot(4, 2, 4);
  foo=[];
  for i=1:length(time)
    foo = [foo bflt_cov(BF_ZD, BF_ZD, i)];
  end
  plot2d(time, foo);
  xtitle('COV ZDZD');

  subplot(4, 2, 6);
  foo=[];
  for i=1:length(time)
    foo = [foo bflt_cov(BF_BIAS, BF_BIAS, i)];
  end
  plot2d(time, foo);
  xtitle('COV BIASBIAS');

  subplot(4, 2, 8);
  foo=[];
  for i=1:length(time)
    foo = [foo bflt_cov(BF_C, BF_C, i)];
  end
  plot2d(time, foo);
  xtitle('COV CC');

  drawnow();

