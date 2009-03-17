


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


