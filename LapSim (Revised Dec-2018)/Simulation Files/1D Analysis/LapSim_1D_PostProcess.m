%% SCRIPT: LapSim_1D_PostProcess

clc
clear
close all

%% Load Data
load('Data_1D_power_accel.mat');

%% Inputs - Change Variable Names of Right Hand Side to Match Source File
x_type = 'Power'; 
x_unit = 'hp (CBR Shape)';
x_GTMS = 79.9;
event = 'Accel';

%% Input - Scenarios (x var first, y var second)
scenarios = {'GTMS F18', x_GTMS};

%% Properly Space Out Input
[x,t,dx] = InputSpacing(x,t);

%% Plot Line Data
figure('units','normalized','outerposition',[0 0 1 1])
plot(x,t); grid minor; hold on
title(['Effect of ' x_type ' on ' event ' Event Time'],'FontSize',25)
xlabel([x_type ' [' x_unit ']'],'FontSize',20)
ylabel('Lap Time [s]','FontSize',20)

%% Find Lap Times for Each Case and Plot
num = size(scenarios,1); lap_t = zeros(1,num); lap_dt = lap_t;
legendnames = cell(1,num);
for i = 1:num
    lap_t(i) = interp1(x,t,scenarios{i,2});
    lap_dt(i) = round(lap_t(i) - lap_t(1),2);
    if i == 1
        legendnames{i} = [scenarios{i,1} ': ' num2str(lap_t(i)) 's'];       
        plot(scenarios{i,2},lap_t(i),'r.','MarkerSize',50)
    else       
        legendnames{i} = [scenarios{i,1} ': ' num2str(lap_t(i)) 's (' num2str(lap_dt(i)) 's)'];
        plot(scenarios{i,2},lap_t(i),'r.','MarkerSize',50)
    end
end

%% Add Legend
leg = legend([{'Line Plot'},legendnames]);
leg.FontSize = 15; leg.Location = 'northeast';

%% Calculate Gradient
[del_x] = gradient(t,dx);

%% Find Gradient at GTMS Point
x_grad = interp1(x,del_x,scenarios{1,2});
x_equiv = round(1/x_grad,3);
str = sprintf('GTMS: 1 s = %g %s',x_equiv,x_unit);
fprintf([str '\n\n']);
text(min(xlim)+0.01*diff(xlim),min(ylim)+0.05*diff(ylim),str,'color','k','FontSize',20)

%% Save
saveas(gcf,['Line Plot - ' x_type ' vs ' event ' Time.jpg'])

%% Function to Space Out to Get Unit Spacing
function [x,t,dx] = InputSpacing(x,t)
    x_unit = linspace(min(x),max(x),1000);
    t_unit = interp1(x,t,x_unit);
    x = x_unit; t = t_unit;
    dx = x(2)-x(1);
end