function [t_brake,d_brake,v_brake]= Solver_Brake
%% FUNCTION: BrakingSolver_V1
%
% Author:        Christian Free
% Modified:      Nov. 19 2018
%
% Purpose:       Find speed trace for a vehicle given peak braking
%
% Misc. Notes:   1) Assume drag acts at CG height
%                2) Assume center of pressure is at 50% wheelbase
%                3) This sim is invalid for heay drag cars (could flip)
%
% Inputs:        None
%
% Outputs:       1) t_brake: vector for time in brake trace in s
%                2) d_brake: vector for distance in brake trace in m
%                3) v_brake: vector for velocity in brake trace in m/s

%% Declare Global Variables
global tuneTTCBraking c  mass Cd Cl frontA L h rho g plotting

%% Find Velocity Over Time
% Drag is the only force that helps you brake (in addition to tires)
drag = @(v) 1/2.*rho.*frontA.*Cd .* v.^2;
downForceCalc = @(v) -1/2.*rho.*frontA.*Cl.*v.^2;

cornerweights= g.*[mass*c/2 mass*c/2;mass*(1-c)/2 mass*(1-c)/2];

tStep = 0.01;  % step for time
v_brake = [40];   % start at 35 m/s
t_brake = 0;  % start at t = 0 s
i = 1;  % counter variable
gForce = -1;    % initial braking deceleration
rearWheelForce = 0;    % Used for Hybrids Class
rearWheelPower = 0;
cumEnergy = 0;
    while v_brake(end) > 0
        decel = gForce(i)*g.*tStep; % change in velocity over the time period
        v_brake(i+1) = v_brake(end)+decel;
        downForce = downForceCalc(v_brake(end));    % total downforce
        maxFrontW = (mass*g+downForce)./2;      % per tire

        dw(i)=(-1*mass*h*g*gForce(i)/L - drag(v_brake(end))*h)/2; % weight transferred longitudinally(kg) per tire
        newweights = cornerweights + [dw(i), dw(i);-dw(i), -dw(i)] + downForce./4;
        newweights = [min(newweights(1,1),maxFrontW),min(newweights(1,2),maxFrontW); newweights(2,1), newweights(2,2)];
        newsideforce=newweights.*tuneTTCBraking*R25Bdriving(newweights,true);
        rearWheelForce(i+1) = sum(newsideforce(2,:));   % Used for Hybrids Class
        rearWheelPower(i+1) = rearWheelForce(i).*v_brake(end);  
            if (v_brake(end)< 34.5) && (v_brake(end)> 15)
                cumEnergy = cumEnergy+ rearWheelPower(i+1).*tStep;
            end    % Used for Hybrids Class
        gForce(i+1)=-1*(sum(newsideforce(:))+drag(v_brake(end)))./(mass.*g); % possible deceleration given weight transfer    
        i = i+1;
        t_brake(i) = t_brake(end) + tStep;   % t is time
    end
%% plot rear wheel power
if plotting
    figure 
         hold on
         title('Rear Wheel Braking Forces')
         yyaxis left
         plot(t_brake,rearWheelForce)
         ylabel('Force [N]')
         yyaxis right
         plot(t_brake,rearWheelPower/1000)
         ylabel('Power [kW]')
         xlabel('Time [s]]')
     fprintf('Energy reclaimed during peak breaking is %f J \n',cumEnergy)
end
%% calculate distance

offsetVel = horzcat(0,v_brake(1:end-1));
avgVel = (v_brake+offsetVel)./2;
offsetT = horzcat(0,t_brake(1:end-1));
deltaT = t_brake-offsetT;
d_brake = cumsum(avgVel.*deltaT);

%% plot
if plotting
     figure
     ylim([0,40]);
     hold on
     title('Braking event')
     xlabel('Time [s]')
     yyaxis left
     plot(t_brake,v_brake)
     plot(t_brake,d_brake)
     yyaxis right
     plot(t_brake,gForce)
     legend('Velocity [m/s]','Distance [m]','gForce');
end

end
