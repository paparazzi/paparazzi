x = 1.12345678910111213;

n = 6;

a(1) = floor(x)
for i = 2:n,
  x = 1 / (x - a(i-1));
  a(i) = floor(x);
end

N = [1 a(1)];
D = [0 1];

for i = 2:n,
  N(i+1) = a(i)*N(i) + N(i-1);
  D(i+1) = a(i)*D(i) + D(i-1);
end

y = N(n+1)/D(n+1);

N
D
printf('%0.12f',y)

