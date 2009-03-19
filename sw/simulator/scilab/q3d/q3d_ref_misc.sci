


function [time, Xref] = get_reference_circle()
   
  radius = 1;
  period = 5;
  dt = 1/512;
  time = 0:dt:period;
  
  omega = 2*%pi/period;
  X0 = radius * sin(omega * time);
  Z0 = radius * (1-cos(omega * time));

  X1 = omega * radius * cos(omega * time);
  Z1 = omega * radius * sin(omega * time);

  X2 = omega * omega * radius * -sin(omega * time);
  Z2 = omega * omega * radius * cos(omega * time);
  
  X3 = omega * omega * omega * radius * -cos(omega * time);
  Z3 = omega * omega * omega * radius * -sin(omega * time);

  X4 = omega * omega * omega * omega * radius * sin(omega * time);
  Z4 = omega * omega * omega * omega * radius * -cos(omega * time);

  Xref = [X0;Z0;X1;Z1;X2;Z2;X3;Z3;X4;Z4];
  
endfunction

//http://www.tsplines.com/resources/class_notes/Bezier_curves.pdf
function [time, Xref] = get_reference_poly(duration, a, b)

  p0 = a(1:2);
  p3 = b(1:2);
  p1 = p0 - a(3:4);
  p2 = p3 - b(3:4);
   
  d0 = p1 - p0;
  d1 = p2 - p1;
  d2 = p3 - p2;
  
  dd0 = d1 - d0;
  dd1 = d2 - d1;

  ddd0 = dd1 - dd0;
  
  dt = 1/512; 
  dt = 1/512;
  time = 0:dt:duration;

  Xref = zeros(10, length(time));
  for i=1:length(time)
    t = time(i);
    Xref(1:2,i) = (1-t)^3*p0 + 3*t*(1-t)^2 * p1 + 3*t^2*(1-t) * p2 + t^3*p3;
    Xref(3:4,i) = 3*((1-t)^2*d0 + 2*t*(1-t) * d1 + t^2 * d2 );
    Xref(5:6,i) = 2*((1-t)*dd0 + t * dd1 );
    Xref(7:8,i) = ddd0;
  end
  
  
endfunction


//http://www.tsplines.com/resources/class_notes/Bezier_curves.pdf
function [time, Xref] = get_reference_poly2(duration, a, b)

  p0_0 = a(1:2)  * (-1)^(9);
  p1_0 = a(3:4)  * (-1)^(8);
  p2_0 = a(5:6)  * (-1)^(7);
  p3_0 = a(7:8)  * (-1)^(6);
  p4_0 = a(9:10) * (-1)^(5);

  p0_1 = p1_0 * 1 + 9 * p0_0;
  p1_1 = p2_0 * 2 + 8 * p1_0;
  p2_1 = p3_0 * 3 + 7 * p2_0;
  p3_1 = p4_0 * 4 + 6 * p3_0;

  p0_2 = p1_1 * 1 + 8 * p0_1;
  p1_2 = p2_1 * 2 + 7 * p1_1;
  p2_2 = p3_1 * 3 + 6 * p2_1;

  p0_3 = p1_2 * 1 + 7 * p0_2;
  p1_3 = p2_2 * 2 + 6 * p1_2;

  p0_4 = p1_3 * 1 + 5 * p0_3;

  p0_9 = b(1:2);
  p1_8 = b(3:4);
  p2_7 = b(5:6);
  p3_6 = b(7:8);
  p4_5 = b(9:10);

  // FIXME
  p0_8 = 1/3*(p1_8 - p0_9);
  p1_7 = 1/3*(p2_7 - p1_8);
  p2_6 = 1/3*(p3_6 - p2_7);
  p3_5 = 1/3*(p4_5 - p3_6);

  p0_7 = 1/4*(p1_7 - p0_8);
  p1_6 = 1/4*(p2_6 - p1_7);
  p2_5 = 1/4*(p3_5 - p2_6);
  
  p0_6 = 1/5*(p1_6 - p0_7);
  p1_5 = 1/5*(p2_5 - p1_6);
  
  p0_5 = 1/6*(p1_5 - p0_6);

  
  p1_4 = p0_4 + p0_5;
  p2_4 = ;
  p2_3 = ;
  
  p3_4 = ;
  p3_3 = ;
  p3_4 = ;
  p3_2 = ;

  p4_4 = ;
  p4_3 = ;
  p4_2 = ;
  p4_1 = ;
  
  dt = 1/512; 
  dt = 1/512;
  time = 0:dt:duration;

  Xref = zeros(10, length(time));
  for i=1:length(time)
    t = time(i);
    Xref(1:2,i)  = p0_0*(1-t)^9 + p0_1*t*(1-t)^8 + p0_2*t^2*(1-t)^7 + p0_3*t^3*(1-t)^6 + p0_4*t^4*(1-t)^5 + p0_5*t^5*(1-t)^4 + p0_6*t^6*(1-t)^3 + p0_7*t^7*(1-t)^2 + p0_8*t^8*(1-t)^1 + p0_9*t^9;
    Xref(3:4,i)  = p1_0*(1-t)^8 + p1_1*t*(1-t)^7 + p1_2*t^2*(1-t)^6 + p1_3*t^3*(1-t)^5 + p1_4*t^4*(1-t)^4 + p1_5*t^5*(1-t)^3 + p1_6*t^6*(1-t)^2 + p1_7*t^7*(1-t)^1 + p1_8*t^8;
    Xref(5:6,i)  = p2_0*(1-t)^7 + p2_1*t*(1-t)^6 + p2_2*t^2*(1-t)^5 + p2_3*t^3*(1-t)^4 + p2_4*t^4*(1-t)^3 + p2_5*t^5*(1-t)^2 + p2_6*t^6*(1-t)^1 + p2_7*t^7;
    Xref(7:8,i)  = p3_0*(1-t)^6 + p3_1*t*(1-t)^5 + p3_2*t^2*(1-t)^4 + p3_3*t^3*(1-t)^3 + p3_4*t^4*(1-t)^2 + p3_5*t^5*(1-t)^1 + p3_6*t^6*;
    Xref(9:10,i) = p4_0*(1-t)^5 + p4_1*t*(1-t)^4 + p4_2*t^2*(1-t)^3 + p4_3*t^3*(1-t)^2 + p4_4*t^4*(1-t)^1 + p4_5*t^5;
  end
  
  
endfunction
