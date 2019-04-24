function [t] = LapSim_DOE_Analysis
clc
clear
close all

%% Note
% Comment out the same global variables in LapSim_Declarations.m
% Change variable names in Inputs Section and within loop

%% Inputs
global massCar Cd
x = (430:2:460)*0.453592; % kg, converted from lb 
y = 0.8:0.1:2; % Cd in m^2
filename = 'LapSim_DOE_Weight_Cd.mat';

%% Go To Parent Folder With Lap Sim
parent_directory = pwd;
while parent_directory(end) ~= '\'
    parent_directory = parent_directory(1:end-1);
end
parent_directory = parent_directory(1:end-1);
original_directory = pwd; cd(parent_directory)

%% Instantiate Matrix
t = zeros(length(y),length(x));

%% Get Results for Each Point
n = 1;
for i = 1:length(y)
    for j = 1:length(x)
        massCar = x(j); Cd = y(i);
        fprintf('DOE Run %g of %g: (%g, %g) \n',n, length(x)*length(y),x(j),y(i))
        t(i,j) = LapSim_FSAEM2018;
        fprintf('\n')
        n = n+1;
    end   
end

%% Go Back to Original Folder and Save Data
cd(original_directory)
save(filename,'x','y','t')

end
