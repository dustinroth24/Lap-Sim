function [rpm, nextRPM, shiftSpeed] = Solver_Shifting(RPMs,torqueEng)
%% FUNCTION: Solver_Shifting
%
% Author:        Christian Free
% Modified:      Dec. 19 2018
%
% Purpose:       Find shift points to maximize wheel torque
%
% Misc. Notes:
%
% Inputs:        1) RPMs: vector of engine speed [rpm]
%                2) torqueEng: vector of engine torque [Nm] at each speed
%
% Outputs:       1) rpm: vector of shift points for each gear
%                2) nextRPM: vector of RPMs engine will go to after shift
%                3) shiftSpeed: vector of car speed at which a shift occurs


%% Declare Global Variables
global tranRatio primaryRatio finalRatio wDiam plotting

%% Instantiate Variables
overallRatio = zeros(1,length(tranRatio)); % overall ratio from engine to wheel in each gear
wheelForce   = zeros(length(RPMs),length(tranRatio)); % wheel force at each rpm and gear
wheelSpeed   = zeros(length(RPMs),length(tranRatio)); % wheel speed at each rpm and gear
rpm          = zeros(1,length(tranRatio)-1); % see outputs
shiftSpeed   = zeros(1,length(tranRatio)-1); % see outputs
nextRPM      = zeros(1,length(tranRatio)-1); % see outputs

%% Solve for Wheel Force and Speed in Each Gear and Plot (Gear Chart)
if plotting
    figure
end
for i = 1:length(tranRatio)
    overallRatio(i) = tranRatio(i)*primaryRatio*finalRatio;
    wheelForce(:,i) = torqueEng*overallRatio(i)/(wDiam/2);  
    wheelSpeed(:,i) = RPMs/(60*overallRatio(i))*(pi*wDiam);
    if plotting
        plot(wheelSpeed(:,i),wheelForce(:,i)); hold on
    end
end

%% Find Curve Intersection (Shift Points)
for i = 1:length(tranRatio)-1
    for j = 1:length(RPMs)
        speed = wheelSpeed(j,i);
        Torque1 = wheelForce(j,i);
        Torque2 = interp1(wheelSpeed(:,i+1),wheelForce(:,i+1),speed);
        if Torque1-Torque2 <0 ||  j == length(wheelSpeed(:,i))
            rpm(i) = wheelSpeed(j,i)*(60*overallRatio(i))/(pi*wDiam);
            shiftSpeed(i) = wheelSpeed(j,i);
            nextRPM(i) = rpm(i)/(tranRatio(i)/tranRatio(i+1));   
            if plotting
                plot(shiftSpeed(i),Torque1,'X');   % plot shift point
            end
            break
        end

    end
end

if plotting
    title('Engine Force per Gear','FontSize',40); grid minor
    xlabel('Wheel Speed [m/s]','FontSize',30); ylabel('Wheel Force [N]','FontSize',30)
    leg = legend('1st','2nd','3rd','4th','5th','6th','shift 1','shift 2','shift 3','shift 4','shift 5'); leg.FontSize = 20;
end

end