clear
close all

% DATA
tmax = 200;
t=1:tmax;
command = zeros(1,tmax);
command(50:150) = 1;

% reference model
vref = zeros(1,tmax);
vrefdot = 0;

sysx = 0.2;
sysvx = 0;
plotsys = zeros(1,tmax);

integrator = 0;
plotint  =zeros(1,tmax);

x = zeros(1,1:tmax);
d = zeros(1,1:tmax);
filt = zeros(1,1:tmax);

% RLS
rls_p = 5;
rls_delta = 1;
rls_lambda = 0.95;
rls_x = ones(rls_p+1, 1).* x(1);
rls_w = ones(rls_p+1, 1)./(rls_p+1);
rls_P = eye(rls_p+1) .* rls_delta;

for i=2:tmax
    acc = (command(i) - vref(i-1)) / 8 - vrefdot * 0.7 ;
    if (acc > 0.02)
        acc = 0.02;
    elseif (acc<-0.02)
        acc = -0.02;
    end
    vrefdot = vrefdot + acc;
    if vrefdot > 0.5
        vrefdot = 0.5
    elseif vrefdot < -0.5
        vrefdot = 0.5
    end
    vref(i) = vref(i-1) + vrefdot;
    
    % Control with Integrator
    error = vref(i) - sysx + rand(1,1) / 20.0;
    integrator = integrator + error / 2;
    syscomm = error * 4 + integrator;
    
    sysacc = (syscomm - (sysx-0.2)*0.75) / 5 - sysvx*0.6;
    if (sysacc > 0.04)
        sysacc = 0.04;
    elseif (sysacc<-0.04)
        sysacc = -0.04;
    end
    sysvx = sysvx + sysacc;
    if sysvx > 0.6
        sysvx = 0.6
    elseif sysvx < -0.7
        sysvx = 0.7
    end
    sysx = sysx + sysvx;
    
    plotsys(i) = sysx;
    plotint(i) = integrator;
end

close all
figure
plot(command , 'r');
hold on
plot(vref,'g');
plot(plotsys,'b')
plot(plotint,'k');

%% Rest


for i=t
   rls_x = [ x(i); rls_x(1:end-1)];
   filt(i) = rls_w' * rls_x;
   rls_alpha = d(i) - filt(i);
   rls_g = rls_P * rls_x / (rls_lambda + rls_x' * rls_P * rls_x);
   rls_P = rls_P / rls_lambda - rls_g*rls_x'* rls_P / rls_lambda;
   rls_w = rls_w + rls_alpha * rls_g;
   
   
end


figure;
plot(orig,'r');
hold on;
plot(echo,'b');
plot(filt,'g');

