function [time,score,place]=skidPadSimp(a,tw)
% Skidpad sim
% Author: Dustin Roth
% Date updated: 7/9/2018
[num,~,~]=xlsread('2018_skidpadResults.xlsx');
r=15.25/2; %radius of skidpad
tw=tw*.0254; %converts track from inches to meters
rc=r+.5*tw+.3048; %the theoretical radius the cg travels
accel=a*9.81;
s=2*pi*rc;
v=sqrt(accel*rc);
time=s/v;
% since the skidpad time is an average of the 2nd and 4th lap, we assume
% them to be the same time and thus only calculate one circle.
Tmin=min(num);
if time<Tmin
    Tmin=time;
end
Tmax=1.25*Tmin;
score=(((Tmax/time)^2)-1)/(((Tmax/Tmin)^2)-1);
score=71.5*score+3.5;
index=time<num;
place=find(index,1);
if score<3.5
    score=3.5;
end
end