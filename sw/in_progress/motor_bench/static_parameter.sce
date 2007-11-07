clear();
getf('mb_utils.sci');

load('averaged_ramp_geared.dat','av_throttle','av_rpm')   
//separating the values to 20% to 90%
start=(length(av_throttle)-1)/5
end=(length(av_throttle)-1)-(length(av_throttle)-1)/10
for i=start:end, sep_av_throttle(i-start+1)=av_throttle(i);end
for i=start:end, sep_av_rpm(i-start+1)=av_rpm(i);end
fitted_courve = param_fit(sep_av_rpm, sep_av_throttle);
	
//xbasc();
//subplot(3,1,1);
//xtitle('Relative Throttle by RPM');
//plot2d(av_throttle,av_rpm);

//subplot(3,1,2);
//xtitle('Fittet Relative Throttle by RPM');
//plot2d(sep_av_throttle,[sep_av_rpm],style=[5,4]);

//subplot(3,1,3);
//xtitle('Fittet Relative Throttle by RPM');
//plot2d(sep_av_throttle,[fitted_courve],style=[4]);
