///
// $Id$
//  
// Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
// 
// This file is part of paparazzi.
// 
// paparazzi is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2, or (at your option)
// any later version.
// 
// paparazzi is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with paparazzi; see the file COPYING.  If not, write to
// the Free Software Foundation, 59 Temple Place - Suite 330,
// Boston, MA 02111-1307, USA. 
///

clear();

M=fscanfMat('traj.out');

value     = M(:,1);
raw_sensor_f = M(:,2);
raw_sensor_i = M(:,3);
scaled_sensor_f = M(:,4);
scaled_sensor_i = M(:,5);
time=1:length(value);


//k = find(time > 500 & time < 550);
k = find(value > -0.009 & value < 0.009);
//k = 1:length(time);


scf();
//clf();

drawlater();

subplot(3,1,1);
xtitle('real', 'time (s)','');
plot2d(time(k), value(k), 3);
plot2d2(time(k), scaled_sensor_i(k)/2^10, 2);

legends(["float", "int", "fixed"],[5 3 2], with_box=%f, opt="lr");

subplot(3,1,2);
xtitle('raw', 'time (s)','');

plot2d(time(k), raw_sensor_f(k), 3);
plot2d2(time(k), raw_sensor_i(k), 2);

legends(["float", "int", "fixed"],[5 3 2], with_box=%f, opt="lr");

subplot(3,1,3);
xtitle('scaled', 'time (s)','');
plot2d(value(k), scaled_sensor_f(k), 3);
plot2d2(value(k), scaled_sensor_i(k), 5);
//plot2d(time(k), zeros(1, length(time(k))), 1);
xgrid(1);


drawnow();
