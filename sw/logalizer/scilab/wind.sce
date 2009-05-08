// $Id$
//
// Wind estimation by analysing aircraft trajectories
//  
// Copyright (C) 2009 ENAC, Pascal Brisset
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

exec wind.sci;

[M, header] = fscanfMat('../../../var/logs/09_05_08__21_38_56:1.csv')

// Column indexes for this log export
//  <entry name="to_export" value="GPS:itow;GPS:speed;GPS:alt;GPS:course" application="log plotter"/>
index_time=1;
index_course=tab_index("GPS:course", header)
index_speed=tab_index("GPS:speed", header)

tmax = M($, index_time)

estimation_period = 30;

// Output wind every period
period=30
for i=1:tmax/period do
  tmin = max(0, i*period - estimation_period)
  x = wind_estimation(M, tmin, i*period)
  printf("%d-%d: %.1f %.1f\n", tmin, i*period, x(1), x(2))
end
