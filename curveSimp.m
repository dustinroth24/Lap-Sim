function [t,v]=curveSimp(g,r,theta)
% Simple lapsim
% Author: Dustin Roth
% Updated: 5/20/2018
% theta is in degrees
rad=theta*pi/180;
% puts angle into radians
arc=r.*rad;
% distance car travels
accel=g*9.81;
% doing this in metric, sorry. Acceleration in m/s/s
v=sqrt(accel*r);
t=arc./v;
% gets time and velocity
end