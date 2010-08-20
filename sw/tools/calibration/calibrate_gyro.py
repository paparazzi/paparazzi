#! /usr/bin/env python

#  $Id$
#  Copyright (C) 2010 Antoine Drouin
#
# This file is part of Paparazzi.
#
# Paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# Paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.  
#

#
# calibrate gyrometers using turntable measurements
#
from optparse import OptionParser

import re
import scipy
from scipy import linspace, polyval, polyfit, sqrt, stats, randn
from pylab import *

import calibration_utils



axis_p = 1
axis_q = 2
axis_r = 3


ac_id = "160"
tt_id = "43"
filename = "/home/poine/work/enac-lara/kahina/scilab/data/crista/log_gyro_r.data"
axis = axis_r

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
# crista
# p : a= 3864.82 b=31288.09, std error= 0.866
# q : a= 3793.71 b=32593.89, std error= 3.070
# r : a= 3817.11 b=32709.70, std error= 3.296
#

samples =  calibration_utils.read_turntable_log(ac_id, tt_id, filename, 1, 7)


#Linear regression using stats.linregress
t  = samples[:,0]
xn = samples[:,axis]
(a_s,b_s,r,tt,stderr)=stats.linregress(t,xn)
print('Linear regression using stats.linregress')
print('regression: a=%.2f b=%.2f, std error= %.3f' % (a_s,b_s,stderr))
print('<define name="GYRO_X_NEUTRAL" value="%d"/>' % (b_s));
print('<define name="GYRO_X_SENS" value="%f" integer="16"/>' % (a_s/pow(2,12)));

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

