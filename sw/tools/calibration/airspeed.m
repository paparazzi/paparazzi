
close all

%% Read Data

% AIR = load('airspeed.txt');
% air = AIR(:,5);
% desired = AIR(:,6);
% climb = AIR(:,7)./100;
% speed = AIR(:,8)./100;
% alt = AIR(:,9)./100;
% course = AIR(:,10)./10;
% 
% airspeed_bias = 64.5;
% airspeed_scale = 3.5;

%% Read data

AIR = load('airspeed_amp.txt');
AIR = AIR(1000:end,:);
air = AIR(:,6);
desired = AIR(:,9)./100;
climb = AIR(:,7)./100;
speed = AIR(:,8)./100;
alt = AIR(:,9)./100;
course = AIR(:,10)./10;

airspeed_bias = 370;
airspeed_scale = 3.5 / 5.7 * 2.4;

%% Altitude

figure
hold on
plot (alt);
plot(desired,'r');
grid on;

%% Wind

windx = 7;
windy = 0;

figure;
vx = speed .* cosd(course) + windx;
vy = speed .* sind(course) + windy;
vz = climb;
plot(vx,vy,'bx');
grid on;
axis equal;

%% Airspeed Calibration

gpsairspeed = sqrt(vx.^2 + vy .^2 + vz.^2);

airsp = air - airspeed_bias;
airsp = airsp - airsp .* (airsp < 0);
airsp = sqrt(airsp) * airspeed_scale;

% [b,a] = butter(2,0.1);
% airsp = filter(b,a,airsp);
% gpsairspeed = filter(b,a,gpsairspeed);

figure
plot(airsp,'b');
hold on;
plot(gpsairspeed,'r');
grid on;
legend('air','gps');
%plot(climb,'g');

%% Total Energy

%Ekin = airsp .*

