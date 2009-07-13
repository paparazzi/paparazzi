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

time    = M(:,1);
fzsp    = M(:,2);
fzdsp   = M(:,3);
fz      = M(:,4);
fzd     = M(:,5);
fzdd    = M(:,6);
iz      = M(:,7);
izd     = M(:,8);
izdd    = M(:,9);


clf();

drawlater();

subplot(3,1,1);
xtitle('z', 'time (s)','');
plot2d(time, fzsp, 1);
plot2d(time, fz, 2);
plot2d(time, iz, 3);
legends(["sp", "float", "int"],[1 2 3], with_box=%f, opt="ur");

subplot(3,1,2);
xtitle('zd', 'time (s)','');
plot2d(time, fzdsp, 1);
plot2d(time, fzd, 2);
plot2d(time, izd, 3);
legends(["float", "int"],[2 3], with_box=%f, opt="lr");

subplot(3,1,3);
xtitle('zdd', 'time (s)','');
plot2d(time, fzdd, 2);
plot2d(time, izdd, 3);
legends(["float", "int"],[2 3], with_box=%f, opt="lr");

drawnow();
