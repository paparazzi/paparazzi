#! /usr/bin/env python

import calibration_utils
import re
import scipy
from scipy import linspace, polyval, polyfit, sqrt, stats, randn
from pylab import *



axis_p = 1
axis_q = 2
axis_r = 3


ac_id = "153"
tt_id = "43"
filename = "data/imu_lisa3/log_gyro_q"
axis = axis_q

#
# lisa 3
# p : a=-4511.16 b=31948.34, std error= 0.603
# q : a=-4598.46 b=31834.48, std error= 0.734
# r : a=-4525.63 b=32687.95, std error= 0.624
#
# lisa 4
# p : a=-4492.05 b=32684.94, std error= 0.600
# q : a=-4369.63 b=33260.96, std error= 0.710
# r : a=-4577.13 b=32707.72, std error= 0.730
#

samples =  calibration_utils.read_turntable_log(ac_id, tt_id, filename, 1, 7)


#Linear regression using stats.linregress
t  = samples[:,0]
xn = samples[:,axis]
(a_s,b_s,r,tt,stderr)=stats.linregress(t,xn)
print('Linear regression using stats.linregress')
print('regression: a=%.2f b=%.2f, std error= %.3f' % (a_s,b_s,stderr))

#
# overlay fited value
#
ovl_omega = linspace(1,7.5,10)
ovl_adc = polyval([a_s,b_s],ovl_omega)

title('Linear Regression Example')
subplot(3,1,1)
plot(samples[:,1])
plot(samples[:,2])
plot(samples[:,3])
legend(['p','q','r']);

subplot(3,1,2)
plot(samples[:,0])

subplot(3,1,3)
plot(samples[:,0], samples[:,axis], 'b.')
plot(ovl_omega, ovl_adc, 'r')

show();

