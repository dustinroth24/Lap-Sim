function [t,vf,tVec,vVec] = Straight_Compiler(Vi,Vf,D,t_accel,d_accel,v_accel,t_brake,d_brake,vel_brake)
%% FUNCTION: StraightSolver_V1
%
% Author:        Christian Free
% Modified:      Dec. 19 2018
%
% Purpose:       Solve for the time it takes to complete a straight, given
%                a vehicle's positive and negative acceleration trace
%
% Misc. Notes:   1) Assume downforce equally divided on all 4 wheels
%                2) No drag contribution to weight transfer
%
% Inputs:        1) Vi: Scalar of initial velocity in m/s
%                2) Vf: Scalar of final velocity in m/s
%                3) D: Scalar of otal distance in m
%                4) t_accel: vector of accel times (from profile)
%                5) d_accel: vector of accel distances (from profile)
%                6) v_accel: vector of accel velocities (from profile)
%                4) t_brake: vector of brake times (from profile)
%                5) d_brake: vector of brake distances (from profile)
%                6) v_brake: vector of brake velocities (from profile)
%
% Outputs:       1) t: scalar of time for straight in s
%                2) vf: scalar of final velocity in straight in m/s
%                3) tVec: vector of time trace corresponding to vVec
%                4) vVec: vector of velocity trace corresponding to tVec

%% Declare Global Variable
global debug_plotting

%% Initial Interpolation of Acceleration and Braking Curves
% Accel
tStart_accel = interp1(v_accel,t_accel,Vi); % time on acceleration curve corresponding to initial speed (corner exit)
dStart_accel = interp1(t_accel,d_accel,tStart_accel); % distance on acceleration curve corresponding to tStart_accel
dFinish_accel = interp1(v_accel,d_accel,Vf); % distance on acceleration curve corresponding to finish speed (corner entry)
tFinish_accel = interp1(d_accel,t_accel,D+dStart_accel); % time on acceleration curve corresponding to finish distance

    vfA = interp1(d_accel,v_accel,D+dStart_accel);

% Braking    
    indB = interp1(vel_brake,t_brake,Vf);   % index for braking
    indDistB = interp1(vel_brake,d_brake,Vf);  % final distance index for braking.
    indDistStartB = interp1(vel_brake,d_brake,Vi); %initial distance index for braking
    vfB = interp1(d_brake,vel_brake,D+indDistStartB);
    
dtoVfA = dFinish_accel-dStart_accel;   % distance to accelerate to Vf
dtoVfB = indDistB-indDistStartB;    % distance to decelerate to Vf

%% Error check
if dtoVfA>D || isnan(dtoVfA)
    %warning('Not enough space to accelerate\n');
    t = tFinish_accel-tStart_accel;   % assume car completes straight and never reaches Vf
    vf = vfA;
    tVec = horzcat(tStart_accel,t_accel(t_accel>tStart_accel & t_accel<tFinish_accel),tFinish_accel) - tStart_accel;
    vVec = horzcat(Vi,v_accel(v_accel>Vi & v_accel<vfA),vfA);
    return
elseif dtoVfB>D
    warning('Not enough space to brake\n');
    tStart = interp1(vel_brake,t_brake,Vi);
    tFinish_accel = interp1(vel_brake,t_brake,vfB);
    t = indB-tStart;    % assume car completes straight in the amount of time it would take to brake from Vi to Vf
    vf = vfB;
    tVec = horzcat(tStart,t_brake(t_brake<tFinish_accel & t_brake>tStart),tFinish_accel) - tStart;
    vVec = horzcat(Vi,vel_brake(vel_brake<Vi & vel_brake>vfB),vfB);
    return
else
%% main body   
        % create masks for each braking and accel event
        mask = (d_accel>dStart_accel) & (d_accel<(dStart_accel+D));   
        d_accel = [dStart_accel, d_accel(mask), dStart_accel+D]; % you have to tack on the additional DistanceStart+D incase you get caught mid shift
        v_accel = [Vi, v_accel(mask), vfA];
        t_accel = [tStart_accel, t_accel(mask), tFinish_accel];
    d_accel=d_accel-dStart_accel;
indB = interp1(vel_brake,t_brake,Vf);   % index for braking
    indDistB = interp1(vel_brake,d_brake,Vf);  % distance index for braking.
        mask = (d_brake<indDistB)& ((d_brake+D-indDistB)>0);
        d_brake = [d_brake(mask) indDistB];
        vel_brake = [ vel_brake(mask), Vf];
        t_brake = [t_brake(mask) indB];
    d_brake = d_brake+D-indDistB;
    
    
    [distanceX,velX] =  intersections(d_accel,v_accel,d_brake,vel_brake);    % finds distance intersection and velocity intersection
    
    % error check
    if isempty(distanceX)
        warning('problem finding brake zone \n Vi = %f \n Vf = %f \n D = %f \n',Vi,Vf,D)
        t = tFinish_accel-tStart_accel;   % assume car completes straight and never reaches Vf
        vf = vfA;
        return
    end
    
    tAEnd = interp1(d_accel,t_accel,distanceX);    % time at acceleration end
    tBStart = interp1(d_brake,t_brake,distanceX);  %time at braking start
    tATotal = tAEnd-tStart_accel;   %total time for acceleration
    tBTotal = indB-tBStart; %total time for braking
    
    t = tATotal+tBTotal;    %final time for straight
    vf = Vf; % final speed, great success
    
%% redo vel and t vectors for plotting and scripts
% process the begining and end of the vectors to account for earlier masks
vVec = [v_accel(v_accel<velX), velX, vel_brake(vel_brake<velX)];
tVecA = [t_accel(t_accel<tAEnd),tAEnd] - tStart_accel;    % orientate to 0
tVecB = t_brake(t_brake>tBStart) - tBStart + (tAEnd-tStart_accel); % orientate to tAEnd
tVec = [tVecA, tVecB];

t_accel = t_accel-t_accel(1); t_brake = t_brake-t_brake(1);   %orient time around the beggining of braking and accel

%% Plotting

if debug_plotting
    figure
    subplot(1,2,1)
        xlim([0,D])
        hold on
        title('Straight Event')
        yyaxis left
        ylabel('Velocity [m/s]')
        xlabel('Distance [m]')
        plot(d_accel,v_accel,d_brake,vel_brake,distanceX,velX,'ok');
        yyaxis right
        ylabel('Time [s]')
        plot(d_accel,t_accel,d_brake,t_brake)
        legend('Accelerating Vel','Braking Vel','Braking Point','Accelerating Time','Braking Time');
    subplot(1,2,2)
        plot(tVec,vVec)
        title('Straight Event Velocity Trace')
        xlabel('Time [s]')
        ylabel('Velocity [m/s]')
end

end

end