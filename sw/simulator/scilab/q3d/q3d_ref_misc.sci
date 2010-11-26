
function [ref_in] = get_reference_circle_entry(center, radius, duration)
  [time_out, ref_out] = get_reference_circle(center, [0], [], radius, duration);
  ref_in = ref_out(:,1);
endfunction


function [time_out, ref_out] = get_reference_circle(time_in, ref_in, center, radius, duration)

  dt = 1/512;
  time = time_in($)+dt:dt:time_in($)+duration;

  omega = 2*%pi/period;
  X0 = radius * sin(omega * time) + center(AXIS_X);
  Z0 = radius * -cos(omega * time) + center(AXIS_Z);

  X1 = omega   * radius * cos(omega * time);
  Z1 = omega   * radius * sin(omega * time);

  X2 = omega^2 * radius * -sin(omega * time);
  Z2 = omega^2 * radius * cos(omega * time);

  X3 = omega^3 * radius * -cos(omega * time);
  Z3 = omega^3 * radius * -sin(omega * time);

  X4 = omega^4 * radius * sin(omega * time);
  Z4 = omega^4 * radius * -cos(omega * time);

  time_out = [time_in time];
  ref_out = [ref_in X0;Z0;X1;Z1;X2;Z2;X3;Z3;X4;Z4];

endfunction



function [time, Xref] = get_reference_looping(t0, duration, center, radius)

  dt = 1/512;
  time = t0:dt:t0+duration;
  delta_dot = %pi/duration;
  delta = delta_dot*time;

  X0 = radius *  sin(2*delta) + center(AXIS_X);
  Z0 = -radius * (cos(2*delta)-1) + center(AXIS_Z);

  X1 = +2*radius*delta_dot.*cos(2*delta);
  Z1 = +2*radius*delta_dot.*sin(2*delta);

  X2 = -4*radius*(delta_dot^2).*sin(2*delta);
  Z2 = +4*radius*(delta_dot^2).*cos(2*delta);

  X3 = -8*radius*(delta_dot^3).*cos(2*delta);
  Z3 = -8*radius*(delta_dot^3).*sin(2*delta);

  X4 = +16*radius*(delta_dot^4).*sin(2*delta);
  Z4 = -16*radius*(delta_dot^4).*cos(2*delta);

  Xref = [X0;Z0;X1;Z1;X2;Z2;X3;Z3;X4;Z4];

endfunction




//http://www.tsplines.com/resources/class_notes/Bezier_curves.pdf
function [time_out, Xref] = get_reference_poly(duration, a, b)

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
 // p2_4 = ;
 // p2_3 = ;

 // p3_4 = ;
 // p3_3 = ;
 // p3_4 = ;
 // p3_2 = ;

 // p4_4 = ;
 // p4_3 = ;
 // p4_2 = ;
 // p4_1 = ;

  dt = 1/512;
  dt = 1/512;
  time = 0:dt:duration;

  Xref = zeros(10, length(time));
  for i=1:length(time)
    t = time(i);
//    Xref(1:2,i)  = p0_0*(1-t)^9 + p0_1*t*(1-t)^8 + p0_2*t^2*(1-t)^7 + p0_3*t^3*(1-t)^6 + p0_4*t^4*(1-t)^5 + p0_5*t^5*(1-t)^4 + p0_6*t^6*(1-t)^3 + p0_7*t^7*(1-t)^2 + p0_8*t^8*(1-t)^1 + p0_9*t^9;
//    Xref(3:4,i)  = p1_0*(1-t)^8 + p1_1*t*(1-t)^7 + p1_2*t^2*(1-t)^6 + p1_3*t^3*(1-t)^5 + p1_4*t^4*(1-t)^4 + p1_5*t^5*(1-t)^3 + p1_6*t^6*(1-t)^2 + p1_7*t^7*(1-t)^1 + p1_8*t^8;
//    Xref(5:6,i)  = p2_0*(1-t)^7 + p2_1*t*(1-t)^6 + p2_2*t^2*(1-t)^5 + p2_3*t^3*(1-t)^4 + p2_4*t^4*(1-t)^3 + p2_5*t^5*(1-t)^2 + p2_6*t^6*(1-t)^1 + p2_7*t^7;
//    Xref(7:8,i)  = p3_0*(1-t)^6 + p3_1*t*(1-t)^5 + p3_2*t^2*(1-t)^4 + p3_3*t^3*(1-t)^3 + p3_4*t^4*(1-t)^2 + p3_5*t^5*(1-t)^1 + p3_6*t^6*;
//    Xref(9:10,i) = p4_0*(1-t)^5 + p4_1*t*(1-t)^4 + p4_2*t^2*(1-t)^3 + p4_3*t^3*(1-t)^2 + p4_4*t^4*(1-t)^1 + p4_5*t^5;
  end

endfunction

function [time_out, ref_out] = get_reference_poly3(time_in, ref_in, duration, state_out)

  if size(ref_in($))~=size(state_out)
   error('get_ref_poly3: boundary conditions not compatible');
  end

  dimension = 2;
  a = [ ref_in(1,$) ref_in(3,$) ref_in(5,$) ref_in(7,$) ref_in(9,$)
        ref_in(2,$) ref_in(4,$) ref_in(6,$) ref_in(8,$) ref_in(10,$) ];
  b = [ state_out(1) state_out(3) state_out(5) state_out(7) state_out(9)
        state_out(2) state_out(4) state_out(6) state_out(8) state_out(10) ];
  dt = 1/512;
  time = dt:dt:duration;

  Coeff = list();
  for i = 1:dimension
    Coeff($+1) = coeff_from_bound(a(i,:)',b(i,:)', duration);
  end

  for i = 1:dimension
    for j = 1:length(time)
      for k = 1:length(Coeff(i))
        ref_out(i+2*(k-1),j) = polyval(Coeff(i)(k),time(j),0,duration);
      end
    end
  end

  time_out = [time_in time];
  ref_out = [ref_in ref_out];
endfunction


function [time_out, ref_out] = get_reference_lti4(time_in, ref_in, duration, pos_out)

  dt = 1/512;
  time_out = time_in;
  ref_out = ref_in;
  for i=1:duration/dt
    time_out = [time_out time_out($)+dt];
    refi = ode(ref_out(1:8,$), time_out($-1), time_out($), list(lti4_get_derivatives, pos_out));
    xdot = lti4_get_derivatives(0, refi, pos_out)
    ref_out = [ref_out [refi; xdot(7:8)]];
  end

endfunction

lti4_omega1 = [ rad_of_deg(35); rad_of_deg(35)];
lti4_zeta1  = [ 0.9; 0.9 ];

lti4_omega2 = [ rad_of_deg(720); rad_of_deg(720)];
lti4_zeta2  = [ 0.9; 0.9 ];

lti4_a0 = lti4_omega1^2 .* lti4_omega2^2;
lti4_a1 = 2 * lti4_zeta1 .* lti4_omega1 .* lti4_omega2^2 + ...
    2 * lti4_zeta2 .* lti4_omega2 .* lti4_omega1^2;
lti4_a2 = lti4_omega1^2 + ...
    2 * lti4_zeta1 .* lti4_omega1 .* lti4_zeta2 .* lti4_omega2 + ...
    lti4_omega2^2;
lti4_a3 = 2 * lti4_zeta1 .* lti4_omega1 + 2 * lti4_zeta2 .* lti4_omega2;

lti4_sat_err = 5;


function [Xdot] =lti4_get_derivatives(t, X, u)

  Xdot(1:2) = X(3:4);
  Xdot(3:4) = X(5:6);
  Xdot(5:6) = X(7:8);
  err_pos = X(1:2) - u;
  err_pos = trim(err_pos, -lti4_sat_err, lti4_sat_err);
  Xdot(7:8) = -lti4_a3 .* X(7:8) -lti4_a2 .* X(5:6) -lti4_a1 .* X(3:4) -lti4_a0.*err_pos;

endfunction




function ref_display(time, ref)

  subplot(5,2,1);
  plot2d(time, ref(1,:));
  xtitle('X(0)');

  subplot(5,2,2);
  plot2d(time, ref(2,:));
  xtitle('Z(0)');

  subplot(5,2,3);
  plot2d(time, ref(3,:));
  xtitle('X(1)');

  subplot(5,2,4);
  plot2d(time, ref(4,:));
  xtitle('Z(1)');

  subplot(5,2,5);
  plot2d(time, ref(5,:));
  xtitle('X(2)');

  subplot(5,2,6);
  plot2d(time, ref(6,:));
  xtitle('Z(2)');

  subplot(5,2,7);
  plot2d(time, ref(7,:));
  xtitle('X(3)');

  subplot(5,2,8);
  plot2d(time, ref(8,:));
  xtitle('Z(3)');

  subplot(5,2,9);
  plot2d(time, ref(9,:));
  xtitle('X(4)');

  subplot(5,2,10);
  plot2d(time, ref(10,:));
  xtitle('Z(4)');

endfunction

