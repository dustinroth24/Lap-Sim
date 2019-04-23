% aeroScript downforce goals for smallest constant turn
clear
clc
c=.5; % weight distributiuon %front
w=600; % weight of car
b=60.5; % wheelbase
t=48; % trackwidth
h=14; % cg height off the ground
tm=t*0.0254;
gTarget=2;
accel=gTarget*9.81;
cornerweights=[w*c/2 w*c/2;w*(1-c)/2 w*(1-c)/2];
diameter=30+tm; % meters, path the car follows
r=diameter/2;
velo=sqrt(accel*r);
mph=velo*2.23694;
dw=(w*gTarget*h/t)/2;
newweights=cornerweights+[-dw dw;-dw dw];
force=abs(min(newweights(:)));
downforce=4*force;
curve=polyfit([0,mph],[0,downforce],2);
clc
str=sprintf('%.0f lbf @ %.0f mph',downforce, mph);
disp(str);
newdownforce=polyval(curve,60);
str=sprintf('%.0f lbf @ 60 mph',newdownforce);
disp(str);




