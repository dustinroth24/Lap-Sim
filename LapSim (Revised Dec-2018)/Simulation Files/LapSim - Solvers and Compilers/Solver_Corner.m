function [time,v,gForce] = Solver_Corner(r,path)
%% FUNCTION: Solver_Corner
%
% Author:        Christian Free
% Modified:      Dec. 19 2018
%
% Purpose:       Calculate car stats for a given steady state corner.  Use
%                weight ransfer and TTC data with a tuning variable
%
% Misc. Notes:   1) Assume downforce equally divided on all 4 wheels
%                2) No drag contribution to weight transfer
%
% Inputs:        1) r: scalar of corner radius in meters
%                2) path: scalar of corner length in meters
%
% Outputs:       1) time: scalar of time to complete corner in s
%                2) v: scalar of the velocity you can corner at in m/s
%                3) gForce: scalar of the gForce you can corner at in gs

%% Declare Global Variables
global rho Cl mass w_track c h g frontA peakSpeed tuneTTCTurning

%% Calculate Downforce and Corner Weights
downForceCalc = @(v) -1/2*rho*frontA*Cl*v^2 ;
cornerweights = g.*mass*[c/2, c/2;(1-c)/2, (1-c)/2];

%% Solve for Maximum Velocity - Increase Velocity Until Not Able to Corner
r = r + w_track/2; % true radius of corner (includes track width)
vStep = .01; % the step size for increasing velocity [m/s]
finish = false; % breaking variable for loop
v = .1; % initial velocity to start testing at [m/s]

while finish == false
    accel = (v^2/r); % centripetal acceleration in m/s^2
    downforce = downForceCalc(v)./4; % downforce per tire (assume equally divided on all 4)    
    dwFront = (mass*c*accel*h/w_track); % mechanical weight transfer, front axle [N]
    dwRear = (mass*(1-c)*accel*h/w_track); % mechanical weight transfer, rear axle [N]
    newweights = cornerweights+[-dwFront,dwFront;-dwRear,dwRear] + downforce; % add downforce and weight transfer to corners
    maxFrontW = mass*c*g+downforce*2; % used to handle flipping - maximum possible (zero force on opposite side)
    maxRearW = mass*(1-c)*g+downforce*2; % used to handle flipping - maximum possible (zero force on opposite side)
    newweights = [max(newweights(1,1),0),min(newweights(1,2),maxFrontW); max(newweights(2,1),0),min(newweights(2,2),maxRearW)];    % if the car is flipping, it can't get more weight
    
    forces = tuneTTCTurning.*R25Bcornering(newweights,true).*newweights; % find forces from frictions times weights
    accel_max_possible = sum(forces(:))/(mass); % find maximum possible acceleration
    v = v + vStep; % step to next velocity;
    if accel > accel_max_possible || v > peakSpeed % see if we can't go any faster
        finish = true; % finish loop
        v = v - vStep; % step back to last possible velocity
        time = path/v; % assume constant speed
        gForce = ((v^2)/r)/g; % calculate g force acceleration
    end
end   
