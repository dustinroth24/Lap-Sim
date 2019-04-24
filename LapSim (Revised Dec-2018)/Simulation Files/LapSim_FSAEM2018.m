function [totalTime] = LapSim_FSAEM2018
%% FUNCTION: LapSim_FSAEM2018
%
% Author:        Christian Free
% Modified:      Nov. 19 2018
%
% Purpose:       Simulate FSAEM performance.  Use it to evaluate platforms
%                and defeat the Germans
%
% Contributors:  Christian Free (F18 Powertrain Lead)
%                  Wrote full final sim, did lots of work throughout
%                Dustin Roth (F18-F19 Suspension Lead)
%                   Built initial lap sim (tire data, weight transfer)
%                Mike Green (F19 Suspension Lead)
%                   Helped Dustin with initial sim
%                Elliot Blotcha (Controls member and de facto F18 CT lead)
%                   Started accel sim used to incorporate powertrain
%
% Misc. Notes:   See documentation folder for full information.  Too much
%                to list here.
%
% Inputs:        None, besides global variables in Declarations file
%
% Outputs:       1) totalTime: scalar of lap time in seconds
%                2) Many plots.  Toggle with the Declarations script

clear
close all

%% Declare Lap Sim Variables
LapSim_Declarations

%% Add Other Folders to Path
addpath(genpath('LapSim - Data Storage and Handling'))
addpath(genpath('LapSim - Solvers and Compilers'))

%% Load Track Profile and Instantiate Vectors
global profile_filename
profile = load(profile_filename); % load length and radii of sections
    l = profile.l; r = profile.r; % both in meters (NaN in r corresponds to straights)

t     = zeros(1,length(l));  % time of each curve/straight
vTurn = zeros(1,length(l));  % velocity of each turn
gs    = zeros(1,length(l)); % gs for each turn
tLog  = cell(1,length(1));   % log for the time arrays for each event
vLog  = cell(1,length(1));   % log for the velocity arrays for each event

%% Load and Modify Torque Curve (Increase Resolution)
global torqFile torqF % access global variables
data = load(torqFile);  torqCurve = data.torqCurve; % read torque curve file
RPMs = torqCurve(1,1):10:torqCurve(end,1); % make new vector of RPMs
if RPMs(end) ~= torqCurve(end,1) % ensure last RPM is redline
    RPMs(end) = torqCurve(end,1);
end
torqEng = interp1(torqCurve(:,1),torqCurve(:,2),RPMs).*torqF; % interpolate and take into account torque factor

%% Find Shift Points with Shift_Points Function
global shiftRPM nextRPM shiftSpeed % access global variables
[shiftRPM, nextRPM, shiftSpeed] = Solver_Shifting(RPMs,torqEng); % custom function to calculate ideal shift points
shiftRPM(end+1) = RPMs(end); % for final gear, stop at redline
nextRPM(end+1) = RPMs(end); % for final gear, stop at redline

%% Create Characteristic Curves for Full Acceleration and Full Braking
[t_accel,d_accel,v_accel] = Solver_Accel(RPMs,torqEng); % vectors of time, distance, and velocity
[t_brake,d_brake,v_brake] = Solver_Brake; % vectors of time, distance, and velocity

%[t_accel,d_accel,v_accel,t_brake,d_brake,v_brake] = straightSim;
global peakSpeed; peakSpeed = max(v_accel) - 1;

%% Solve for Turns, Initially Assuming Entry Speed Can Be Met - Maximum Possible Velocity in Each Corner
for i = 1:length(r) % loop through each corner
    if ~isnan(r(i)) % if it is a corner (not a straight)
        [t(i),vTurn(i),gs(i)] = Solver_Corner(r(i),l(i)); % use the corner solver to find time, velocity, and acceleration
        tLog{i} = [0, t(i)]; % add time to tLog for speed trace
        vLog{i} = [vTurn(i), vTurn(i)]; % add time to vLog for speed trace
            % at this point, assume corner entry and exit speeds are the same
    end
end

%% Solve For Straights, Using Characteristic Curves
vEntry = [2.2,vTurn,peakSpeed+1];
for i = 1:length(r)
    if isnan(r(i))
        [t(i),vEntry(i+1),tLog{i},vLog{i}] = Straight_Compiler(vEntry(i),vEntry(i+2),l(i), ...
                                             t_accel,d_accel,v_accel,t_brake,d_brake,v_brake);
    end
end

%% Make Sure Corner Entry Speed was Met By Straight - Accelerate or Brake in Corner

for i = 1:length(r) % for each segment
    if ~isnan(r(i)) % if it is a corner (else disregard)
        %% CASE:  Too Slow Going into Corner
        if (vTurn(i)-vEntry(i))>0.01 % if the last straight/turn was too slow
            tStart = interp1(v_accel,t_accel,vEntry(i)); % find the time on accel curve corresponding to entry velocity
            tFinish = interp1(v_accel,t_accel,vTurn(i)); % find the time on accel curve corresponding to turn velocity
            dStart = interp1(v_accel,d_accel,vEntry(i)); % find the distance on accel curve corresponding to entry velocity
            dFinish = interp1(v_accel,d_accel,vTurn(i)); % find the distance on accel curve corresponding to turn velocity
            tAccel = tFinish-tStart; dAccel = dFinish-dStart; % subtract to find times and distances for acceleration
            
            %% Edit and Complete Rest of Corner - Two Cases: dAccel > l or dAccel <= l
            if dAccel> l(i)
                %warning('Vehicle did not reach corner exit speed on corner %.0f.\n Length of corner: %.3f m \n Length needed: %.3f m \n',i, l(i), dAccel)
                vEntry(i+1) = interp1(d_accel,v_accel,l(i)+dStart);
                    % Log traces
                    tFinish = interp1(v_accel,t_accel,vEntry(i+1));
                    t(i) = tFinish-tStart;
                    tLog{i} = [tStart, t_accel(t_accel>tStart & t_accel<tFinish), tFinish] - tStart;
                    vLog{i} = [vEntry(i), v_accel(v_accel>vEntry(i) & v_accel<vEntry(i+1)), vEntry(i+1)];
                if isnan(r(i+1))
                    % rerun straight solver for next straight
                    [t(i+1),vEntry(i+1+1),tLog{i+1},vLog{i+1}] = Straight_Compiler(vEntry(i+1),vEntry(i+3),l(i+1), ...
                                                                 t_accel,d_accel,v_accel,t_brake,d_brake,v_brake);
                end
            else
                % Log Traces
                t(i) = tAccel + (l(i)-dAccel)/vTurn(i); % calculate final time from vTurn
                tLog{i} = [tStart, t_accel(t_accel>tStart & t_accel<tFinish), tFinish] - tStart;
                tLog{i} = [tLog{i}, t(i)];
                vLog{i} = [vEntry(i), v_accel(v_accel>vEntry(i) & v_accel<vTurn(i)), vTurn(i), vTurn(i)];
            end
        %% CASE:  Too Fast Going into Corner
        elseif (vTurn(i)-vEntry(i))<0.1  % if the last straight/turn was too fast
            tStart = interp1(v_brake,t_brake,vEntry(i));   % index for deceleration: find the time at which the initial velocity is
            dStart = interp1(v_brake,d_brake,vEntry(i));
            dFinish = interp1(v_brake,d_brake,vTurn(i));
            tFinish = interp1(v_brake,t_brake,vTurn(i));
            tAccel = tFinish-tStart; dAccel = dFinish-dStart;
                % Log Traces
                t(i) = tAccel + (l(i)-dAccel)/vTurn(i); % calculate final time from vTurn
                tLog{i} = [tStart, t_brake(t_brake>tStart & t_brake<tFinish), tFinish] - tStart;
                tLog{i} = [tLog{i}, t(i)];
                vLog{i} = [vEntry(i), v_brake(v_brake<vEntry(i) & v_brake>vTurn(i)), vTurn(i), vTurn(i)];
        end
    end
end

%% Post-Process: Display Lap Time and Plot Metrics - Velocity Trace, Cornering Data, Tire Data
global plotting % declare global variable
totalTime = sum(t);
fprintf('Total time for course: %.3f s\n',totalTime);

if plotting
    % Compile and Plot Detailed Velocity Trace
    while length(tLog) > 1 
        tLog{1} = [tLog{1}, tLog{2} + tLog{1}(end) + 1e-4];   % concatenate and add time to sync
        tLog(2) = []; % delete entry 2
        vLog{1} = [vLog{1},vLog{2}]; % concatenate
        vLog(2) = []; % delete entry 2
    end
    tLog = cell2mat(tLog); vLog = cell2mat(vLog); % convert from cell to simple vector
    figure(6); plot(tLog,vLog); grid minor; 
    title('Velocity Trace for the Event','FontSize',40)
    xlabel('Time [s]','FontSize',30); ylabel('Velocity [m/s]','FontSize',30)

    % Calculate and Plot Cornering Data for Different Radii
    r = linspace(2,50);
    time = zeros(1,length(r)); v = zeros(1,length(r)); gForce = zeros(1,length(r));
    for i = 1:length(r)
       [time(i),v(i),gForce(i)]= Solver_Corner(r(i),10);
    end
    figure(7)
    subplot(2,1,1); plot(r,v); grid minor; title('Cornering Prediction','FontSize',40)
    xlabel('Radius corner [m]','FontSize',30); ylabel('Velocity [m/s]','FontSize',30)
    subplot(2,1,2); plot(r,gForce); grid minor
    xlabel('Radius corner [m]','FontSize',30); ylabel('G-Force [g]','FontSize',30)

    % Plot Tire Data (Friction vs Load)
    loads = linspace(0,500);  mu = R25Bcornering(loads);
    figure(8); plot(loads,mu); title('Tire Loading','FontSize',40); grid minor
    xlabel('Tire force [lbf]','FontSize',30); ylabel('Ceofficient of Friction [-]','FontSize',30)
end

end