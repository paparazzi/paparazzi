function P = coeff_from_bound(a,b,d_time)
  
  d = length(a)-1;
  N = 2*d+1;
  if d+1~=length(b)
    error('coeff_from_bound:'...
	  +' boundary conditions not compatible');
  else
    P = list();
    A = lin_sys(d);
    for i = 1:d+1
      a(i) = d_time^(i-1)*(-1)^(N-i+1)*a(i);
      b(i) = d_time^(i-1)*b(i);
    end
    p_0_init = (A\a)';
    p_0_end = (A\b)';
    p_0_end = p_0_end(:,$:-1:1);
    P($+1) = cat(2,p_0_init,p_0_end);
    for i = 1:d
      P($+1) = deriv_coeff(P($),i,N);
    end
    for i = 1:d
      P(i+1) = d_time^(-i)*P(i+1);
    end
  end
    
endfunction

function p_d = deriv_coeff(p,d,n)

  for i = 1:length(p)-1
    p_d(i) = i*p(i+1)+(n-(d+i)+2)*p(i);
  end
  
endfunction

function res = arr(n,m)

  if m>n
    error('arr: could not compute arrangement')
  else
    res = factorial(n)/factorial(n-m);
  end  
  
endfunction

function M = lin_sys(d)

  n = 2*d+1;
  for i = 1:d+1
    for j = 1:d+1
      if j<=i
	M(i,j) = arr(i-1,j-1)*arr(n-j+1,i-j);
      else
	M(i,j) = 0;
      end
    end
  end
    
endfunction

function res = polyval(p,t,t0,tf)

  // Coordinate change
  u = (t-t0)/(tf-t0);
  
  // Normalized Polynomial Value
  res = 0;
  n = length(p)
  for i = 1:n
    res = res + p(i)*u^(i-1)*(u-1)^(n-i);
  end
  
endfunction

function present(P)

  d = length(P);
  t = linspace(0,1,50);
  for i = 1:length(t)
    for j = 1:d
      RES(j,i) = polyval(P(j),t(i));
    end
  end
  
  clf();
  drawlater();
  for i = 1:d
    subplot(d,1,i)
    plot2d(t,RES(i,:),2);
    xgrid();
  end
  drawnow();
  
endfunction



