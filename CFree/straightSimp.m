function [t]=straightSimp(a,b,L,v0,vf)
% Simple lapsim: Straights
% Author: Dustin Roth
% Updated: 7/5/2018
accel=a*9.81;
brake=b*9.81;
% separates the straight into two portions, one accelerating and one
% braking portion
x1=(vf.^2-v0.^2+2*brake.*L)./(2*(accel+brake));
% result of some algebra to find the acceleration distance
vm=sqrt(v0.^2+2*accel.*x1);
t1=abs((vm-v0)./accel);% time accelerating
t2=abs((vm-vf)./brake);% time braking
t=t1+t2;
end



