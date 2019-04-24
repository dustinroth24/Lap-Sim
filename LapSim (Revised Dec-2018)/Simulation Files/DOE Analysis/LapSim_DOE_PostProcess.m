%% SCRIPT: LapSim_DOE_PostProcess

clc
clear
close all

%% Load Data
load('Lapsim_DOE_Weight_Cd_Imp.mat');

%% Inputs - Change Variable Names of Right Hand Side to Match Source File
x_type = 'Weight'; 
x_unit = 'lb';
x_GTMS = 440;

y_type = 'Drag Coefficient'; 
y_unit = 'Point CD';
y_GTMS = 1.12;

event = 'Autocross';

%% Input - Scenarios (x var first, y var second)
scenarios = {'GTMS F19 Wt, F19 Cd', x_GTMS, y_GTMS};

%% Properly Space Out Input
[x,y,t,dx,dy] = InputSpacing(x,y,t);

%% Find Lap Times for Each Case
num = size(scenarios,1); lap_t = zeros(1,num); lap_dt = lap_t;
[x, y] = meshgrid(x, y);
for i = 1:num
   lap_t(i) = round(interp2(x,y,t,scenarios{i,2},scenarios{i,3}),2);
   lap_dt(i) = round(lap_t(i) - lap_t(1),2);
end

%% Plot Contour Data
tfast = ceil(min(min(t)))-1; tslow = floor(max(max(t)))+1; range = tfast:1:tslow;
figure('units','normalized','outerposition',[0 0 1 1])
[c_pts, ~] = contourf(x,y,t,range,'Showtext','on'); grid on; grid minor; hold on
title(['Effect of ' x_type ' and ' y_type ' on ' event ' Event Time'],'FontSize',25)
xlabel([x_type ' [' x_unit ']'],'FontSize',20)
ylabel([y_type ' [' y_unit ']'],'FontSize',20)

%% Plot Individual Circles and Prepare for Legend
legendnames = cell(1,num);
for i = 1:num
    if i == 1
        legendnames{i} = [scenarios{i,1} ': ' num2str(lap_t(i)) 's'];       
        plot(scenarios{i,2},scenarios{i,3},'r.','MarkerSize',50)
    else       
        legendnames{i} = [scenarios{i,1} ': ' num2str(lap_t(i)) 's (' num2str(lap_dt(i)) 's)'];
        plot(scenarios{i,2},scenarios{i,3},'.','MarkerSize',50)
    end
end

%% Add Legend
leg = legend([{'Contour Plot'},legendnames]);
leg.FontSize = 15; leg.Location = 'northeast';

%% Calculate Gradient
[del_x,del_y] = gradient(t,dx,dy);

%% Find Gradient at GTMS Point
x_grad = interp2(x,y,del_x,scenarios{1,2},scenarios{1,3});
y_grad = interp2(x,y,del_y,scenarios{1,2},scenarios{1,3});
x_equiv = round(1/x_grad,3); y_equiv = round(1/y_grad,3); equiv = round(y_equiv/x_equiv,3);
str = sprintf('GTMS: 1 s = %g %s = %g %s (1 %s = %g %s)',x_equiv,x_unit,y_equiv,y_unit,x_unit,equiv,y_unit);
fprintf([str '\n\n']);
text(min(xlim)+0.01*diff(xlim),min(ylim)+0.05*diff(ylim),str,'color','w','FontSize',20)

%% Save
saveas(gcf,['Contour Plot - ' x_type ' vs ' y_type ' - ' event ' Time.jpg'])

%% Function to Space Out to Get Unit Spacing
function [x,y,t,dx,dy] = InputSpacing(x,y,t)
    if any((diff(x) - mean(diff(x)))~= 0) || ...
       any((diff(y) - mean(diff(y)))~= 0)
        x_unit = linspace(min(x),max(x),100);
        y_unit = linspace(min(y),max(y),100);
        t_unit = zeros(length(y_unit),length(x_unit));
        for i = 1:length(y_unit)
            for j = 1:length(x_unit)
                t_unit(i,j) = interp2(x,y,t,x_unit(j),y_unit(i));
            end
        end
        x = x_unit; y = y_unit; t = t_unit;
    end
    dx = x(2)-x(1); dy = y(2)-y(1);
end