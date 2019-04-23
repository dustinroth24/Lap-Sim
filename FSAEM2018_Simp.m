function FSAEM2018_Simp(gs,car)
% 2018 Dynamic Event simple sim
% Author: Dustin Roth & Mike Green
% Date updated: 8/22/2018
% g is a 1x2 vector of the accelerations
a=gs(1);
b=gs(2);
% accelerating, braking and cornering g's
[num,~,~] = xlsread('AutoXtime_MI_2018.xlsx'); %insert whatever file name you want it to read
timeData=num;%re-writes the data to a variable

%this stuff is used in the calc of your autoX score
tMin=min(timeData);%finds min of the data
tMax=1.45*tMin;%this is your theoretical max
%% radii and angle of curves
r=[115 32 47 51 169 10 31 6 29 63 14 39 46 20 22 25 36 46 85 34 27 87 13 89 10 42 17 16 39 9 121 36 22 52 21 32 88 46 56 69 56 383 32 14 25 16 10 43 74]*0.3048;
% radii of all of the curves
theta=[35.17 72.36 43.49 59.58 16.3 115.93 76.55 140.95 51.83 19.28 63.41 47.71 48.49 29.28 80.04 74.33 52.24 49.45 49.41 71.06 81.88 49.38 128.11 58.4 75.05 44.07 77.7 92.4 149.17 134.44 43.37 35.24 82.51 44.69 94.69 51.61 38.55 56.58 27.68 32.08 37.32 20.62 55.86 30.79 52.72 81.05 103.59 38.32 10.26]; % Degrees
tcurves=zeros(1,length(theta)); % time of each curve
curveMat=zeros(length(theta),3);
gs=zeros(1, length(tcurves));
for i=1:length(tcurves)
    [t,v,g]=turnconaero(r(i),theta(i)*pi/180,car);
    tcurves(i)=t;
    curveMat(i,3)=v;
    gs(i)=g;
end
% mask=gs>=2;
% fast_curves=sum(mask);
% maxc=max(gs);
% open curveMat

%% straights
l=[41,89,49,45,50,56.5,23.5,47.5,14.6,17.2,21.9,55.9,43.9,48.2,23.3,67.5,16,17.2,19.1,73.5,56.9]'*0.3048;
v0=[curveMat(9,3),curveMat(10,3),curveMat(11,3),curveMat(13,3),curveMat(14,3),curveMat(22,3),curveMat(24,3),curveMat(25,3),curveMat(27,3),curveMat(28,3),curveMat(29,3),curveMat(36,3),curveMat(38,3),curveMat(39,3),curveMat(40,3),curveMat(41,3),curveMat(44,3),curveMat(45,3),curveMat(46,3),curveMat(47,3),curveMat(48,3)]';
vf=[curveMat(10,3),curveMat(11,3),curveMat(12,3),curveMat(14,3),curveMat(15,3),curveMat(23,3),curveMat(25,3),curveMat(26,3),curveMat(28,3),curveMat(29,3),curveMat(30,3),curveMat(37,3),curveMat(39,3),curveMat(40,3),curveMat(41,3),curveMat(42,3),curveMat(45,3),curveMat(46,3),curveMat(47,3),curveMat(48,3),curveMat(49,3)]';
%             everything is a column vector
ts=straightSimp(a,b,l,v0,vf); %gets times on the straights

% final straight
lengthF=87.6*0.3048;
v0f=curveMat(end,3);
time=roots([.5*(a*9.81),v0f,-lengthF]);
timeF=max(time);

totalTime=sum(tcurves)+sum(ts)+timeF;
%% scores
tYour=totalTime;%write the found time to a new variable to make calc a little easier for me (mike since i am a stupid man)
mask=totalTime<timeData;%creates a mask to find all of the times that are slower than ours
position=find(mask, 1 );%finds what position you would be in. find just finds the index of the first true value of the mask therefore giving you your place
if tYour<tMin
    tMin=tYour;
end    
autoXscore=118.5*(((tMax/tYour)-1)/((tMax/tMin)-1))+6.5;%calc for your autoX score
if autoXscore<6.5
    autoXscore=6.5;
end

[~,velo]=turnconaero(15.25/2,2*pi,car);
cornering=velo^2/15.25/2;
[skidTime,skidScore,skidPosition]=skidPadSimp(cornering,t);
clc
%% Results
AutoXresults=sprintf('AutoX\nTime: %.3f\nPosition: %.0f\nScore: %.3f\n',totalTime,position,autoXscore);
disp(AutoXresults);

SkidResults=sprintf('Skidpad\nTime: %.3f\nPosition: %.0f\nScore: %.3f\n',skidTime,skidPosition,skidScore);
% disp(SkidResults);
% [aTime,aScore,aPosition]=accelSimp(a);

aResults=sprintf('Acceleration\nTime: %.3f\nPosition: %.0f\nScore: %.3f\n',aTime,aPosition,aScore);
disp(aResults);

total=aScore+skidScore+autoXscore;
totstr=sprintf('Total Dynamic Points: %.3f',total);
% disp(totstr)
% disp(fast_curves);
end