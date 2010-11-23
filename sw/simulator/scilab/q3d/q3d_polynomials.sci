//
// traj is a n_compo*n_order*n_sample vector
// n_compo components
// n_order succesive time derivatives
// n_sample time values
//
function poly_display_traj(time, traj)

  [n_compo, n_order, n_sample] = size(traj);

  for compo=1:n_compo
    for order=1:n_order
      subplot(n_order, n_compo, compo+(order-1)*n_compo);
      plot2d(time, matrix(traj(compo,order,:), n_sample, 1));
      xtitle(sprintf('$X^{%d}_{%d}$', order-1, compo));
    end
  end

endfunction


//
// compute the values of a set of polynomials along a time vector
//
//
function [traj] = poly_gen_traj(time, coefs)

  [n_comp, n_order, n_coef] = size(coefs);

  traj = zeros(n_comp, n_coef/2, length(time));
  for compo=1:n_comp
    for order=1:n_order
      for i=1:length(time)
	traj(compo, order, i) = ...
	    poly_compute_val(matrix(coefs(compo,order, :),1,n_coef), time(1), time(i));
      end
    end
  end

endfunction


//
// compute v = a_{n}*(t-t_0)^{n} + a_{n-1}*(t-t_0)^{n-1} + ... + a_{0}
//
//
function [v] = poly_compute_val(coefs, t0, t)
  dt = t-t0;
  v = coefs($);
  for i=1:length(coefs)-1
    v = v * dt;
    v = v + coefs(length(coefs)-i); // assume coef(1) = a_0
  end

endfunction


//
//  compute coefficients for a set of polynomials
//  having bond values b0 and b1
//
//
function [coefs] = poly_get_coef_from_bound(time, b0,b1)

  [n_comp, n_order] = size(b0);
  n_coef = 2*n_order
  coefs = zeros(n_comp, n_order, n_coef);

  // refer to paper for notations

  for compo=1:n_comp

    // invert of the top left corner block
    invA1 = zeros(n_order, n_order);
    for i=1:n_order
      invA1(i,i) = 1/Arr(i-1,i-1);
    end
    // first half of the coefficients
    coefs(compo, 1, 1:n_order) = (invA1*b0(compo,:)')';

    // bottom left block : triangular
    A3 = zeros(n_order, n_order);
    dt = time($) - time(1);
    for i=1:n_order
      for j=i:n_order
	A3(i,j) = Arr(i-1,j-1) * dt^(j-i);
      end
    end
    // bottom right block
    A4 = zeros(n_order, n_order);
    for i=1:n_order
      for j=1:n_order
	A4(i,j) = Arr(i-1,j-1+n_order) * dt^(j+n_order-i);
      end
    end
    coefs(compo, 1, n_order+1:2*n_order) = ...
	(inv(A4)*(b1(compo,:)' - A3*matrix(coefs(compo,1, 1:n_order), n_order, 1)))';
    // fill in the coefficients for the succesive time derivatives
    for order=2:n_order
      for pow=0:2*n_order-order
	coefs(compo, order, pow+1) = Arr(order-1,pow-1+order)*coefs(compo, 1, pow+order);
      end
    end
  end

endfunction


//
// Arrangement
//
//
function [akn] = Arr(k,n)
  akn = factorial(n)/factorial(n-k);
endfunction
