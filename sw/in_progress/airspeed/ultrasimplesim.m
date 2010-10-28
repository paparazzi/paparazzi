clear
clc
close all



dt = 1/60;
simsteps = 16000;

timespan=0:1/60:100;

t = 0;
airspeed = 14;
acceleration = 0;
vclimb = 0;
altitude = 100;
pitch = 0;
alpha = 0;
pitch_sp = 0.2;
throttle = 0.5;
power = 0.25;
elevator = 0;


% Airspeed command
airspeed_sp  = ones(1,simsteps).* 11;
airspeed_sp(1,6000:12000) = 16;
airspeed_sp(1,13000:14000) = 7;

% Altitude command
altitude_sp  = ones(1,simsteps).* 100;
altitude_sp(1,2000:4000) = 125;
altitude_sp(1,8000:10000) = 125;

battery_good = ones(1,simsteps);
battery_good(1,15000:16000) = 0;

% Save state
X = zeros(simsteps,13);

% Very sofisticated Aircraft Model
Vmax    = 26;   % Level flight full power
Vmin    = 8;    % Stall
Mass    = 1.0;

% Controller States
pitch_cumsum = 0;
airspeed_cumsum = 0;

controller = 1;

for i=1:simsteps
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Simplified A/C long dynamics with stall and energy
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Pitch loop without integrators
    elevator = (pitch_sp - pitch) * 15;
    if (elevator > 0.4)
        elevator = 0.4;
    elseif (elevator < -0.2)
        elevator = -0.2;
    end
    % Follow elevator but also align with the flow
    pitch = pitch + elevator * dt - alpha * dt; 
    % Angle of attack from 0 deg at vmax to 17 deg stall at vmin
    if airspeed > Vmax
        alpha = 0;
        LoverD = 6;
    elseif airspeed < Vmin
        % from Vmin downto 3/4Vmin ... increase alpha from 17 to 45 deg 
        lin = (Vmin-airspeed) / (Vmin/4);
        if (lin > 2)
            lin = 2;
        end
        alpha = (17 + (45-17) * lin) / 57; % complete stall
        LoverD = 8 + (4.1-8) * lin;
    else
        alpha = (Vmax - airspeed) / (Vmax-Vmin) * 17.0 / 57;
        LoverD = 8;
    end
    % Path angle
    gamma = pitch - alpha;
    vclimb = sin(gamma) * airspeed;
    altitude = altitude + vclimb * dt;
    % Using the pitch angle...
    P_climb = vclimb * Mass;
    % Lift over Drag
    P_drag = airspeed / LoverD * Mass;
    % Motor Energy
    if battery_good(i) == 0
        throttle = 0;
    end
    power_sp = throttle ^ 2;
    power = power + (power_sp - power) * 0.1;
    % Full throttle = same energy as vmax glide
    P_motor = (Vmax / LoverD) * sqrt(power) * Mass;
    % Total energy
    P_tot = P_motor - P_drag - P_climb;

    acceleration = P_tot / Mass;
    airspeed = airspeed + (acceleration * dt);
    
    if (airspeed < Vmin/2)
        airspeed = Vmin/2;
    end
    if (altitude < 0)
        altitude = 0;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Run Control 1
    
    if controller == 1
        Throttle_min = 0.2;
        Throttle_max = 0.9;
        
        Pitch_min = -20/57;
        Pitch_max = 30/57;

        

        % Traditional with integrators:

        vclimb_sp = (altitude_sp(i) - altitude);
        if (vclimb_sp > 3)
            vclimb_sp = 3;
        elseif (vclimb_sp < -3)
            vclimb_sp = -3;
        end
        throttle = vclimb_sp * 0.33;
        if (throttle > Throttle_max)
            throttle = Throttle_max;
        elseif (throttle < Throttle_min)
            throttle = Throttle_min;
        end
        % P-gain
        pitch_sp = 0.1 +  vclimb_sp * 0.05;
        if (pitch_sp > Pitch_max)
            pitch_sp = Pitch_max;
        elseif (pitch_sp < Pitch_min)
            pitch_sp = Pitch_min;
        end 

    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Run Control 2
    
    elseif controller == 2
        Throttle_min = 0.15;
        Throttle_max = 1;
        
        Pitch_min = 6/57;
        Pitch_max = 18/57;

        

        % Traditional with integrators:

        vclimb_sp = (altitude_sp(i) - altitude);
        if (vclimb_sp > 3)
            vclimb_sp = 3;
        elseif (vclimb_sp < -3)
            vclimb_sp = -3;
        end
        % I-gain
        pitch_cumsum = pitch_cumsum + vclimb_sp * 0.001;
        if (pitch_cumsum > Pitch_max)
            pitch_cumsum = Pitch_max;
        elseif (pitch_cumsum < Pitch_min)
            pitch_cumsum = Pitch_min;
        end 
        % P-gain
        pitch_sp = pitch_cumsum +  vclimb_sp * 0.05;

        err_airspeed = airspeed_sp(i) - airspeed;
        airspeed_cumsum = airspeed_cumsum + err_airspeed * 0.0002;
        if (airspeed_cumsum > Throttle_max)
            airspeed_cumsum = Throttle_max;
        elseif (airspeed_cumsum < Throttle_min)
            airspeed_cumsum = Throttle_min;
        end
        throttle = airspeed_cumsum + err_airspeed * 0.08;
        if (throttle > Throttle_max)
            throttle = Throttle_max;
        elseif (throttle < Throttle_min)
            throttle = Throttle_min;
        end
    
    end    
    
    % Save For plotting
    X(i,:) = [  airspeed_sp(i) airspeed altitude_sp(i) altitude  throttle power pitch_sp pitch alpha gamma elevator vclimb_sp vclimb]; 
    
end



figure
subplot(6,1,1)
hold on
plot(airspeed_sp,'r');
plot(X(:,2),'b');
plot(ones(simsteps,1)*Vmin,'k');
plot(ones(simsteps,1)*Vmax,'k');
title('Airspeed')
grid

subplot(6,1,2)
hold on
plot(X(:,7)*57,'r');
plot(X(:,8)*57,'b');
title('Pitch')
grid

subplot(6,1,3)
hold on
plot(X(:,5),'r');
title('Throttle')
grid


subplot(6,1,4)
hold on
plot(X(:,9)*57,'g');
plot(ones(simsteps,1)*17,'r');
plot(X(:,10)*57,'k');
title('Alpha Gamma (degrees)')
%legend('1','2')
grid

subplot(6,1,5)
hold on
plot(X(:,12),'r');
plot(X(:,13),'b');
title('VClimb')
grid

subplot(6,1,6)
hold on
plot(altitude_sp,'r');
plot(X(:,4),'b');
title('Altitude')
grid

