x0 = %pi/180.;

x = x0;
n = 20;
m = 31;

a(1) = floor(x);
for i = 2:n,
  x = 1 / (x - a(i-1));
  a(i) = floor(x);
end

N = [1 a(1)];
D = [0 1];

for i = 2:n,
  N(i+1) = a(i)*N(i) + N(i-1);
  D(i+1) = a(i)*D(i) + D(i-1);
  if N(i+1)/D(i+1) == x0,
    break
  end
  if N(i+1) > 2^m | D(i+1) > 2^m,
    N(i+1) = N(i);
    D(i+1) = D(i);
    break
  end
end

y = N(i+1)/D(i+1);

//N
//D
printf('%0.20f, delta=%e, %d',y,y-x0,i)
printf('%20f',N(i+1))
printf('%20f',D(i+1))

