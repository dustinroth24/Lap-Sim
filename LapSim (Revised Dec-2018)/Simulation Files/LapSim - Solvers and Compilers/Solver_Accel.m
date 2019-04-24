function [t_accel,d_accel,v_accel]= Solver_Accel(RPMs,torqEng)
%% FUNCTION: SpeedSolver_V1
%
% Author:        Christian Free
% Modified:      Dec. 19 2018
%
% Purpose:       Find acceleration speed trace for a vehicle given a torque
%                curve
%
% Misc. Notes:   1) Assume downforce equally divided on all 4 wheels
%                2) No drag contribution to weight transfer
%
% Inputs:        1) RPMs: vector of engine speeds in RPM
%                2) torqEng: vector of engine torque in Nm for each RPM
%
% Outputs:       1) t_accel: vector for time in accel trace in s
%                2) d_accel: vector for distance in accel trace in m
%                3) v_accel: vector for velocity in accel trace in m/s

%% assumptions
global tuneTTCBraking tranRatio finalRatio primaryRatio c shiftSpeed mass wDiam t_shift Cd Cl frontA  L h La rho mu massFWheel inertiaEng g effTran shiftRPM nextRPM plotting

% does not include drag moment in weight transfer
%% solver per gear

% tractive effort resistance (sum of drag and rolling resistance
FtResistance = @(v) 1/2.*rho.*frontA.*Cd .* v.^2 + mass.*g.*0.01.*(1+v./160.*3.6);
% a = @(v) (massF*mass)/(tractive(v) - FtResistance(v)); % formula for acceleration

Ft = [];    % Ft is tractive force
V = [];
start = 1;  % starting RPM for next gear
for i = 1:length(tranRatio)
    stop = find(round(RPMs) == round(shiftRPM(i)));    % stopping threshold for current gear
    ratio = tranRatio(i) * primaryRatio*finalRatio; % gear ratio
    V = horzcat(V,((wDiam/2)/ratio *2*pi/60) .* RPMs(start:stop));  % velocity as a function of RPM
    Ft = horzcat(Ft,(ratio*effTran/(wDiam/2)) .* torqEng(start:stop));    % Ft as a function of RPM and then a function of velocity
    start= find(round(RPMs)== roundn(nextRPM(i),1)); 
end
Ft = Ft-FtResistance(V);

if plotting
    figure
    plot(V,Ft,'+')
    title('Wheel Force [N] per Wheel Speed [m/s]')
    hold on
end

FtOriginal = Ft;    % save Tractive force original values, because we will modify them later

%% max wheel force per Velocity
downForce = @(v) -1/2.*rho.*frontA.*Cl.*v.^2;

Ftmax = mu*(mass*g+downForce(V))*(La-.013*h)/L / (1-mu*h/L);   % max tractive effot, negligable, assumes constant rolling resistance
Ft(Ft>Ftmax) = Ftmax(Ft>Ftmax);
if plotting
    plot(V,Ft,'--');
    legend('Possible Force','Traction Limited Force, only Downforce')
end

%% find vel per time

v_accel = [2.2];   % start at 2.2 m/s
t_accel = 0;
tStep = 0.01;
tEvents = 2000; %counter for time steps
j = 1;  % counter for shifts
massF = 1+massFWheel+inertiaEng.*(tranRatio(j).*finalRatio.*primaryRatio).^2;  % mass factor of engine
shiftSpeed(end+1) = 100;    % give a final shift speed that the vehicle will never reach
for i = 2:tEvents
    t_accel(i) = t_accel(end) + tStep;   % t is time, which is a function of i
    v_accel(i) = v_accel(end)+(t_accel(end)-t_accel(end-1))*interp1(V,Ft,v_accel(end))/(mass*massF);
    if v_accel(end)> shiftSpeed(j)      % shift the car when appropriate
       t_accel(end)=t_accel(end)+t_shift;
       j = j+1;
       massF = 1+massFWheel+inertiaEng.*(tranRatio(j).*finalRatio.*primaryRatio).^2; %change inertia for gear shift
    end
    if isnan(v_accel(end))
        v_accel(end) = [];
        t_accel(end) = [];
        break
    end
end


%% account for weight transfer
% This will iterate a couple times until the velocity traces are pretty
% reasonable

cornerweights= g.*[mass*c/2 mass*c/2;mass*(1-c)/2 mass*(1-c)/2]; %[N]

et = 2;   % relative error

while et>0.01
    vel1 = v_accel; % the iteration going into this calculation
    
    accel = diff(v_accel)./diff(t_accel);    %calculate longitudinal acceleration
    
    accel2 = zeros(1,length(accel));   % preallocate space for accel2
    for i = 1:length(accel)
            accel1 = accel(i);  % current acceleration
            dw=mass*h*accel1/2/L; % weight transferred longitudinally(N) per tire
            newweights = cornerweights+[-dw,-dw;dw,dw] + downForce(v_accel(i))/4;    % add downforce to each tire
            newsideforce=newweights.*tuneTTCBraking*R25Bdriving(newweights,true);
            accel2(i)=sum(newsideforce(2,:))/mass; % possible acceleration given weight transfer
    end
    accel2 = horzcat([0], accel2);
    Ftmax = accel2.*mass;  % maximum tractive effot based on acceleration (F=ma), within vel/time domain
    Ftmax = interp1(v_accel,Ftmax,V);   % maximum possible Ft, Ft/V domain.

    Ft = FtOriginal;    %reload the old plot
    Ft(Ft>Ftmax) = Ftmax(Ft>Ftmax);
    
    %% redo Velocity versus time calculation

    v_accel = [2.2];   % start at 2.2 m/s
    t_accel = 0;
    j = 1;  % counter for shifts
    massF = 1+massFWheel+inertiaEng.*(tranRatio(j).*finalRatio.*primaryRatio).^2;
    shiftSpeed(end+1) = 100;    % give a final shift speed that the vehicle will never reach
    for i = 2:tEvents
        t_accel(i) = t_accel(end) + tStep;   % t is time, which is a function of i
        v_accel(i) = v_accel(end)+(t_accel(end)-t_accel(end-1))*interp1(V,Ft,v_accel(end))/(mass*massF);
        if v_accel(end)> shiftSpeed(j)      % shift the car when appropriate
           t_accel(end)=t_accel(end)+t_shift;
           massF = 1+massFWheel+inertiaEng.*(tranRatio(j).*finalRatio.*primaryRatio).^2;
           j = j+1;
        end
        if isnan(v_accel(end))      % if we've reached the end of our velocity vector (peak engine speed)
            v_accel(end) = [];
            t_accel(end) = [];
            break
        end
    end
    vel2 = v_accel;
    
    et = abs((sum(vel2)-sum(vel1))/sum(vel2));   %relative error
end

%% calculate distance

offsetVel = horzcat(0,v_accel(1:end-1));
avgVel = (v_accel+offsetVel)./2;
offsetT = horzcat(0,t_accel(1:end-1));
deltaT = t_accel-offsetT;
d_accel = cumsum(avgVel.*deltaT);

%% tack on running at peak vehicle speed
t_accel(end+1) = t_accel(end)+5;
v_accel(end+1) = v_accel(end)+0.01;
d_accel(end+1) = v_accel(end)*(t_accel(end)-t_accel(end-1))+d_accel(end);


%% plot
if plotting
    plot(V,Ft);
    legend('Possible Force','Traction Limited Force, only Downforce','Weight Transfer and Downforce')

    figure(3)
    hold on
    yyaxis left
    xlabel('Time [s]')
    plot(t_accel,d_accel)
    title('Acceleration Event')
    yyaxis right
    plot(t_accel,v_accel)
    legend('Distance [m]','Velocity [m/s]')
end
end
