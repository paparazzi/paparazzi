clear
clc
close all



dt = 1/60;
simsteps = 25000;

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


% Altitude command
altitude_sp  = ones(1,simsteps).* 100;
% Airspeed command
airspeed_sp  = ones(1,simsteps).* 12;

% slow climb + descend
altitude_sp(1,2000:4000) = 125;
% fast climb + descend
airspeed_sp(1,6000:12000) = 16;
altitude_sp(1,8000:10000) = 125;
% too low airspeed
airspeed_sp(1,13000:14000) = 7;
% too high airspeed
airspeed_sp(1,15000:16000) = 30;
% throttle kill batlow
battery_good = ones(1,simsteps);
battery_good(1,24000:25000) = 0;
% Roll perturbation
roll_perturbation = zeros(1,simsteps);
% roll oscillation: e.g. poor nav
roll_perturbation(1,21000:24000) = abs(sin((21000:24000)./170 .* 6.28));
% Headwind changes by making turns
% -to keep a constant airspeed, a kinematic acceleration is needed
% -or with constant kinetic energy, a change in airspeed is seen
wind = zeros(1,simsteps);
% 60Hz 70m circle 11m/s 410m = ca 2000 samples
wind(1, 17000:23000) = sin((17000:23000)./2000 .* 6.28) .* 5;
% derivative of sin is cos so we might also just turn the wind 90deg
headwind_induced_kinematic_acceleration = wind .* 0.10;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save states for plotting
X = zeros(simsteps,15);
extra1 = 0;
extra2 = 0;

% Very ultra-sofisticated Aircraft Model
Vmax    = 26;   % Level flight full power
Vmin    = 8;    % Stall
Mass    = 1.0;

% Controller States
pitch_cumsum = 0;
airspeed_cumsum = 0;

% RLS
rls_p = 3;
rls_delta = 0;
rls_x = zeros(rls_p+1, 1);
rls_w = zeros(rls_p+1, 1);
rls_P = eye(rls_p+1) .* rls_delta;


for i=1:simsteps
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Simplified A/C long dynamics with stall and energy
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Pitch loop without integrators
    pitch_neutral = 0.2;
    elevator = (pitch_neutral + pitch_sp - pitch) * 15;
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
        % from Vmin downto 0.95 Vmin ... increase alpha from 17 to 45 deg 
        lin = (Vmin-airspeed) / (0.05*Vmin);
        if (lin > 1)
            lin = 1;
        end
        alpha = (17 + (60-17) * lin) / 57; % complete stall
        LoverD = 8 - 7 * lin;
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
    P_drag = airspeed / (LoverD / (1+roll_perturbation(i))) * Mass;
    % Motor Energy
    power_sp = throttle ^ 2;
    power = power + (power_sp - power) * 0.1;
    % Full throttle = same energy as vmax glide
    P_motor = (Vmax / LoverD) * sqrt(power) * Mass;
    % Total energy
    P_tot = P_motor - P_drag - P_climb;

    acceleration = P_tot / Mass + headwind_induced_kinematic_acceleration(i);
    airspeed = airspeed + (acceleration * dt);
    
    if (airspeed < Vmin/2)
        airspeed = Vmin/2;
    end
    if (altitude < 0)
        altitude = 0;
    end
    
    
    controller = 4;


    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Run Control 1: Standard no-airspeed
    
    if controller == 1
        Throttle_min = 0.0;
        Throttle_max = 1.;
        
        Pitch_min = -30/57;
        Pitch_max = 45/57;

        VClimb_max = 3;
        

        % Traditional with integrators:

        vclimb_sp = (altitude_sp(i) - altitude);
        if (vclimb_sp > VClimb_max)
            vclimb_sp = VClimb_max;
        elseif (vclimb_sp < -VClimb_max)
            vclimb_sp = -VClimb_max;
        end
        % throttle increment only....
        throttle = 0.65 + vclimb_sp * 0.9;
        if (throttle > Throttle_max)
            throttle = Throttle_max;
        elseif (throttle < Throttle_min)
            throttle = Throttle_min;
        end
        % pitch of vz
        pitch_sp = 0.025 +  vclimb_sp * 0.05;
        if (pitch_sp > Pitch_max)
            pitch_sp = Pitch_max;
        elseif (pitch_sp < Pitch_min)
            pitch_sp = Pitch_min;
        end 

    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Run Control 2-3: Old Airspeed Loops
    % 2: with saturated integrator as most people used
    % 3: with active integrator
    
    elseif (controller == 2) || (controller == 3)
        Throttle_min = 0.0;
        Throttle_max = 1;
        
        Pitch_min = -6/57;
        Pitch_max = 15/57;

        VClimb_max = 1.0;
        
        

        % Traditional with integrators:

        vclimb_sp = (altitude_sp(i) - altitude);
        if (vclimb_sp > VClimb_max)
            vclimb_sp = VClimb_max;
        elseif (vclimb_sp < -VClimb_max)
            vclimb_sp = -VClimb_max;
        end
        % I-gain
        pitch_cumsum = pitch_cumsum + (vclimb_sp - vclimb) * 0.0015;
        if (pitch_cumsum > Pitch_max)
            pitch_cumsum = Pitch_max;
        elseif (pitch_cumsum < Pitch_min)
            pitch_cumsum = Pitch_min;
        end 
        % P-gain
        pitch_sp = pitch_cumsum +  vclimb_sp * 0.025;
        if (pitch_sp > Pitch_max)
            pitch_sp = Pitch_max;
        elseif (pitch_sp < Pitch_min)
            pitch_sp = Pitch_min;
        end 

        err_airspeed = airspeed_sp(i) - airspeed;
        if (controller == 2)
            airspeed_cumsum = 0.45;
        else
            airspeed_cumsum = airspeed_cumsum + err_airspeed * 0.001;
        end
        if (airspeed_cumsum > Throttle_max)
            airspeed_cumsum = Throttle_max;
        elseif (airspeed_cumsum < Throttle_min)
            airspeed_cumsum = Throttle_min;
        end
        throttle = airspeed_cumsum + err_airspeed * 0.2;
        if (throttle > Throttle_max)
            throttle = Throttle_max;
        elseif (throttle < Throttle_min)
            throttle = Throttle_min;
        end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Run Control 4: Feed Forward Gains to climb/accelerate Tcr+Tcl
    
    elseif controller == 4
        Throttle_min = 0.0;
        Throttle_max = 1;
        
        Pitch_min = -12/57;
        Pitch_max = 25/57;

        if (airspeed_sp(i) <= 12)
            VClimb_max = 3;
            Pitch_max = 10.5/57;
            Pitch_min = -4.5/57;
        else
            VClimb_max = 2;
            Pitch_max = 3.5/57;
            Pitch_min = -11/57;
        end
        
        % Climb feed forward gain per m/s
        Tcl = 0.25;
        % Cruise feed forward gain per m/s
        Tcr = 1/26;

        % Traditional with integrators:

        vclimb_sp = (altitude_sp(i) - altitude);
        if (vclimb_sp > VClimb_max)
            vclimb_sp = VClimb_max;
        elseif (vclimb_sp < -VClimb_max)
            vclimb_sp = -VClimb_max;
        end
        % I-gain
        pitch_cumsum = pitch_cumsum + (vclimb_sp - vclimb) * 0.0015;
        if (pitch_cumsum > Pitch_max)
            pitch_cumsum = Pitch_max;
        elseif (pitch_cumsum < Pitch_min)
            pitch_cumsum = Pitch_min;
        end 
        % P-gain
        pitch_sp = pitch_cumsum +  vclimb_sp * 0.025;
        if (pitch_sp > Pitch_max)
            pitch_sp = Pitch_max;
        elseif (pitch_sp < Pitch_min)
            pitch_sp = Pitch_min;
        end 

        err_airspeed = airspeed_sp(i) - airspeed;
        airspeed_cumsum = airspeed_cumsum + err_airspeed * 0.0005;
        if (airspeed_cumsum > Throttle_max)
            airspeed_cumsum = Throttle_max;
        elseif (airspeed_cumsum < Throttle_min)
            airspeed_cumsum = Throttle_min;
        end
        throttle = airspeed_cumsum + err_airspeed * 0.2 + Tcr * airspeed_sp(i) + Tcl * vclimb_sp;
        if (throttle > Throttle_max)
            throttle = Throttle_max;
        elseif (throttle < Throttle_min)
            throttle = Throttle_min;
        end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Run Control 5: Feed Forward Gains to climb/accelerate Tcr+Tcl
    
    elseif controller == 5
        Throttle_min = 0.0;
        Throttle_max = 1;
        
        Pitch_min = -6/57;
        Pitch_max = 15/57;

        VClimb_max = 1.0;
        
        if (i==1)
            % Climb feed forward gain per m/s
            Tcl = 0.25;
            % Cruise feed forward gain per m/s
            Tcr = 1/2000;
        end

        % Traditional with integrators:

        vclimb_sp = (altitude_sp(i) - altitude);
        if (vclimb_sp > VClimb_max)
            vclimb_sp = VClimb_max;
        elseif (vclimb_sp < -VClimb_max)
            vclimb_sp = -VClimb_max;
        end
        % I-gain
        pitch_cumsum = pitch_cumsum + (vclimb_sp - vclimb) * 0.0015;
        if (pitch_cumsum > Pitch_max)
            pitch_cumsum = Pitch_max;
        elseif (pitch_cumsum < Pitch_min)
            pitch_cumsum = Pitch_min;
        end 
        % P-gain
        pitch_sp = pitch_cumsum +  vclimb_sp * 0.025;
        if (pitch_sp > Pitch_max)
            pitch_sp = Pitch_max;
        elseif (pitch_sp < Pitch_min)
            pitch_sp = Pitch_min;
        end 

        err_airspeed = airspeed_sp(i) - airspeed;
        if (abs(vclimb_sp) > 0.5)        
            %Tcl = Tcl + err_airspeed/airspeed_sp(i) * 0.05 * 0.01;
            Tcr = Tcr + err_airspeed/airspeed_sp(i) * 0.05 * 0.001;
        else
            %Tcl = Tcl + err_airspeed/airspeed_sp(i) * 0.05 * 0.001;
            Tcr = Tcr + err_airspeed/airspeed_sp(i) * 0.05 * 0.01;
        end
        throttle = err_airspeed * 0.2 + Tcr * airspeed_sp(i) + Tcl * vclimb_sp;
        if (throttle > Throttle_max)
            throttle = Throttle_max;
        elseif (throttle < Throttle_min)
            throttle = Throttle_min;
        end
    
        extra1 = Tcr;
    end
    
    % Autopilot actions
    if battery_good(i) == 0
        throttle = 0;
    end
    
    % Save For plotting
    X(i,:) = [  airspeed_sp(i) airspeed altitude_sp(i) altitude  throttle power pitch_sp (pitch-pitch_neutral) alpha gamma elevator vclimb_sp vclimb extra1 extra2]; 
    
end



figure
subplot(6,1,1)
hold on
plot(airspeed_sp,'r');
plot(X(:,2),'b');
plot(ones(simsteps,1)*Vmin,'k');
plot(ones(simsteps,1)*Vmax,'k');
title('Airspeed')
axis([0 simsteps Vmin-1 Vmax+1]);
grid

subplot(6,1,2)
hold on
plot(altitude_sp,'r');
plot(X(:,4),'b');
title('Altitude')
axis([0 simsteps (min(altitude_sp) - 10) (max(altitude_sp)+10)]);
grid

subplot(6,1,3)
hold on
plot(X(:,5),'r');
title('Throttle')
axis([0 simsteps 0 1]);
grid


subplot(6,1,4)
hold on
plot(X(:,7)*57,'r');
plot(X(:,8)*57,'b');
title('Pitch (degrees)')
axis([0 simsteps (Pitch_min*57-3) (Pitch_max*57 + 3)]);
grid

subplot(6,1,5)
hold on
plot(X(:,9)*57,'g');
plot(ones(simsteps,1)*17,'r');
plot(X(:,10)*57,'k');
axis([0 simsteps -10 20]);
title('Alpha (green) Gamma (black) (degrees)')
%legend('1','2')
grid

subplot(6,1,6)
hold on
plot(X(:,12),'r');
plot(X(:,13),'b');
title('VClimb')
axis([0 simsteps (-VClimb_max -0.5) (VClimb_max + 0.5)]);
grid

set(gcf,'PaperUnits','centimeter')
set(gcf,'PaperSize',[21 30.5])
set(gcf,'PaperPosition',[1 1 19 28.5])
print(gcf,'-depsc2', ['results_' num2str(controller)]);
%print(gcf,'-djpeg', ['results_' num2str(controller)]);


