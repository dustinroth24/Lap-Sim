function [t] = LapSim_1D_Analysis
clc
clear
close all

%% Note
% Comment out the global variable in LapSim_Declarations.m
% Change variable name in Inputs section and within loop

%% Inputs
global torqF
x = 0.2:0.1:5; % kg
filename = 'Data_1D_torque_accel.mat';

%% Go To Parent Folder With Lap Sim
parent_directory = pwd;
while parent_directory(end) ~= '\'
    parent_directory = parent_directory(1:end-1);
end
parent_directory = parent_directory(1:end-1);
original_directory = pwd; cd(parent_directory)

%% Instantiate Matrix
t = zeros(1,length(x));

%% Get Results for Each Point
for i = 1:length(x)
    torqF = x(i);
    fprintf('1D Run %g of %g: (%g) \n',i, length(x), x(i))
    t(i) = LapSim_FSAEM2018;
    fprintf('\n')  
end

%% Go Back to Original Folder and Save Data
cd(original_directory)
save(filename,'x','t')
end
