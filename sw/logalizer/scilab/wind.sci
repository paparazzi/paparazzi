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

// c.f. sw/ground_segment/tmtc/wind.ml


// *************************************************************
// Minimisation of a cost function using the Nelder Mead simplex function
// http://en.wikipedia.org/wiki/Nelder-Mead_method
// Implementation for 2D
// 
// Usage:
// [fx,x,num_iter] = NelderMead(init, f, max_iter, precision)
//   - init     : initial point
//   - f        : cost function to minimize
//   - max_iter : maximal number of iterations
//   - precision: simplex size criteria for stopping
//  Returns
//   - x : optimum found
//   - fx : cost of the function for the optimum found
//   - num_iter : actual number of iterations
//
function [fx,x,num_iter] = NelderMead(init, f, max_iter, precision)
  init_size = 2 // Size of the initial simplex
  alpha = 1
  gamma_ = 2
  rho = 0.5
  sigma = 0.5

  // Initial silplex
  x1 = init
  x2 = init+[init_size 0]
  x3 = init+[0 init_size]

  // Table of vectors [f(x) x]
  xs = [ [f(x1), x1]; [f(x2), x2]; [f(x3), x3] ]
  
  num_iter = 0
  while num_iter < max_iter & norm([xs(1,1) xs(3,1)]) > precision do
    num_iter = num_iter + 1

    // Sorting the simplex summits according to the cost function
    xs = gsort(xs, 'lr', 'i')
    
    // Barycenter of all the points except the last one
    x0 = mean([xs(1,2:3); xs(2,2:3)], 'r')
    
    // Reflection
    xr = x0 + alpha * (x0 - xs(3,2:3))
    fxr = f(xr)
    if xs(1,1) <= fxr & fxr < xs(2,1) then
      xs(3, :) = [fxr, xr] // Keep the reflected point
    elseif fxr < xs(1,1) then
      // Expansion
      xe = x0 + gamma_ * (x0 - xs(3,2:3))
      fxe = f(xe)
      if fxe < fxr
	xs(3,:) = [fxe, xe] // Keep the expanded point
      else
	xs(3,:) = [fxr, xr] // Keep the reflected point
      end
    else
      // Contraction
      xc = xs(3, 2:3) + rho * (x0 - xs(3,2:3))
      fxc = f(xc)
      if fxc <= xs(3, 1) then
	xs(3,:) = [fxc, xc] // Keep the contracted point
      else
	// Reduction
	x1 = xs(1, 2:3)
	x2 = x1 + sigma * (xs(2, 2:3) - x1)
	x3 = x1 + sigma * (xs(3, 2:3) - x1)
	xs(2,:) = [f(x2), x2]
	xs(3,:) = [f(x3), x3]
      end
    end
  end
  
  // Returns the best point
  x = xs(1,2:3)
  fx = xs(1,1)
endfunction


pi = acos(-1);


// Compute the mean airspeed from a log file for a given wind
// Usage:
//   [mean_,stdev_] = airspeed(M, tmin, tmax, wind)
//   - M          : the matrix of the log lines
//   - tmin, tmax : time bounds for the computation
//   - wind       : 2d vector to evaluation
function [mean_,stdev_] = airspeed(M, tmin, tmax, wind)
  [nl,n_] = size(M)
  s=0
  s2=0

  i = 1

  // Skip the first lines until tmin
  while M(i, index_time) < tmin do i=i+1; end
  n = i
  
  // Handle the lines until tmax (or end of matrix)
  while i <= nl & M(i, index_time) <= tmax do
    course = M(i,index_course) / 10 / 180 * pi // radian, CW
    speed = M(i,index_speed) / 100 // m/s
    air_speed = [speed*sin(course), speed*cos(course)] - wind
    s = s + norm(air_speed)
    s2 = s2 + norm(air_speed)^2
    i=i+1
  end
  n = i - n
  mean_ = s / n
  stdev_ = sqrt(mean_^2 + (s2-2*s^2/n) / n)
endfunction


// ********************** Wind estimation ********************************
// Usage:
// [x] = wind_estimation(M, tmin, tmax)
//  - M : the matrix of the log lines
//  - tmin, tmax : time bounds for the computation

function [x] = wind_estimation(M, tmin, tmax)
  function [r] = cost(w)
    [m,r] = airspeed(M, tmin, tmax, w)
  endfunction
  [fx,x,n] = NelderMead([0,0], cost, 50, 0.01)
endfunction


// Get the index of index of a field in tab separated line
function [ind] = tab_index(field, line)
  char_ind = strindex(line, field)
  ind = 1;
  temp = ascii(line)
  for i=1:char_ind do
    if temp(i) == 9 then
      ind = ind + 1
    end
  end
endfunction
