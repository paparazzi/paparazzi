clear
close all

% DATA
t=1:200;
orig = sin(t./3)+sin(t./100.*2.*pi);
echo = [orig(4:end) 0 0 0]./1.2 + 0.5;
echo = echo + randn(size(orig))./10;
x = echo;
d = orig;

% RLS
rls_p = 5;
rls_delta = 1;
rls_lambda = 0.95;
rls_x = ones(rls_p+1, 1).* x(1);
rls_w = ones(rls_p+1, 1)./(rls_p+1);
rls_P = eye(rls_p+1) .* rls_delta;

filt = x;

for i=t
   rls_x = [ x(i); rls_x(1:end-2); 1];
   filt(i) = rls_w' * rls_x;
   rls_alpha = d(i) - filt(i);
   rls_g = rls_P * rls_x / (rls_lambda + rls_x' * rls_P * rls_x);
   rls_P = rls_P / rls_lambda - rls_g*rls_x'* rls_P / rls_lambda;
   rls_w = rls_w + rls_alpha * rls_g;
   
   
end


close all;
figure;
plot(orig,'r');
hold on;
plot(echo,'b');
plot(filt,'g');

