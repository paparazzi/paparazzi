clear();
getf('mb_utils.sci');

load('averaged_ramp_geared.dat','av_throttle','av_rpm')   

// extract only data around operating point
//i_s = 20;
//i_e = 90;
i_s = 5;
i_e = 90;
t_part = av_throttle(i_s:i_e);
r_part = av_rpm(i_s:i_e);

// tau_kq kv dv
p0=[0.00001; 20000];
Z = [r_part; t_part];
//[p, err] = datafit(err_mot_lin, Z, p0); 

[p, err] = datafit(err_mot_ric, Z, p0) 
//p = p0;

xbasc();
//subplot(3,1,1);
xtitle('Rpms vs Throttle');
plot2d(av_throttle,av_rpm);
plot2d(t_part,r_part, 4);


r_fit = [];
for i=1:length(av_throttle)
  //  r_fit = [r_fit mot_lin(av_throttle(i), p)];
  r_fit = [r_fit mot_ric_stat(av_throttle(i), p)];
end

plot2d(av_throttle,r_fit, 3);