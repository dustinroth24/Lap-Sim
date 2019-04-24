%% SCRIPT: LapSim_Declarations
%
% Author:        Christian Free (F18 PT Lead)
% Modified:      Nov. 18 2018
%
% Purpose:       Declare variables for the simulation.  Use this as the
%                inputs page to run parameter sweeps.
%
% Misc. Notes:   1) Torque curve must be a .mat file with 1 variable -
%                   torqCurve.  It must be an Nx2 matrix.
%                   Column 1: Engine Speed in RPM, Column 2 : Torque in Nm 
%                   First and last RPM must be divisible by 10
%                   First RPM must be less than 2 m/s wheel speed (rollout)
%                2) Inertias are somewhat guesses, but making them too
%                   large or too small can hae weird effects.  Otherwise,
%                   they do not matter too much.
%                3) profile_filename variable consists of two column
%                   vectors - l and r.  l is segment length in meters and r
%                   is segment radii in meters.  NaN is used for straights.

%% Simulation and Setup Parameters
global profile_filename plotting debug_plotting
profile_filename = 'FSAEM_AutoX_2018.mat';     % .mat file of driing profile
plotting = false;                              % whether or not to display plots [logical]
debug_plotting = false;                        % whether or not to display debug plots [logical]


%% Overall Vehicle Parameters
global massCar c L La w_track h wDiam g mu     % declare global variables
%massCar = 207.2072;                                 % mass of car [kg]
massDriver = 75;                               % mass of driver [kg]
L = 60.5*.0254;                                % wheelbase [m] - converted from in
w_track = 48.*0.0254;                          % trackwidth [m] - converted from in
h = 13.5*.0254;                                  % CG height [m] (aero acts here) - converted from in
c = .47;                                       % fraction weight distribution towards front [-]
La = L*(1-c);                                  % length from front wheel to CG [m]
wDiam = 18.1*.0254;                            % diameter of wheel [m] - converted from in
g = 9.81;                                      % gravity [m/s^2]
mu = 1.0;                                      % coefficient of friction with ground

%% Drivetrain Parameters
global tranRatio primaryRatio finalRatio ...
       inertiaEng t_shift effTran              % declare global variables
tranRatio = [33/12, 32/16, 30/18, ...
             26/18, 30/23, 29/24];             % transmission ratios for each gear [-]
primaryRatio = 2.11;                           % primary reduction for all gears [-]
finalRatio = 43/11;                            % final drive ratio [-] 
inertiaEng = 1.08e-5;                          % inertia of engine [kg-m^2] - measured from test
inertiaWheel = 2*0.137;                        % inertia of wheels [kg-m^2] - 4 wheels and tires
inertiaDrive = 0.05;                           % inertia of driveline [kg-m^2] - no chain
t_shift = .1;                                  % shift time [s] - estimate
effTran = 0.9;                                 % transmission efficiency [-]

%% Aero Parameters
global rho frontA Cd Cl                        % declare global variables
rho = 1.225;                                   % density of air [kg/m^3]
frontA = 1.17;                                 % frontal profile area [m^2]
Cl = -2.557;                                   % lift coefficient [-]
    % CONVENTION: Downforce is negative, lift is positive
%Cd = 1.14;                                     % drag coefficient [-]

%% Data Inputs and Modification
global torqFile torqF tuneTTCTurning ...
       tuneTTCBraking                          % declare global variables
torqFile = 'CBR600RR_80hp.mat';                % torque curve file (see note)
torqF = 1;                                     % torque factor: mutliplier for engine torque
tuneTTCTurning = .75;                          % fraction of mu we typically see from TireTestConsortium Data
tuneTTCBraking = 0.4;                          % fraction of mu we typically see from TireTestConsortium Data

%% Calculated values
global mass massFWheel                         % declare global variables
mass = massCar + massDriver;                   % total vehicle mass [kg]
inertia = inertiaWheel+inertiaDrive;           % total inertia after engine [kg-m^2]
massFWheel= inertia/(mass * (wDiam/2)^2);      % mass factor variable that accounts for driveline intertia and engine inertia. I'm not sure the engine inertia term is correct.