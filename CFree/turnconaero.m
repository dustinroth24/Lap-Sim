function [time,v,g]=turnconaero(r,theta,carinfo)
% car info is c, w, b, t, h
aeromap=polyfit([0,26.8224],[0,340],2);
tm=carinfo(4)*0.0254;
r=r+tm/2;
path=r*theta;
c=carinfo(1);
h=carinfo(5);
t=carinfo(4);
w=carinfo(2);
cornerweights=[w*c/2 w*c/2;w*(1-c)/2 w*(1-c)/2];
finish=false;
v=.1; %this is m/s
i=0;

maxFrontWeight = sum(cornerweights(1,:));   %used to handle flipping. The maximum weight one front tire can have
maxRearWeight = sum(cornerweights(2,:));
while finish==false
    accel=(v^2/r)/9.81;
    dw=(w*accel*h/t)/2; % weight transfer
    downforce=polyval(aeromap,v)/4;
%     downforce=0; % for when I want to ignore aero for a sec
    newweights = [max(newweights(1,1),0),min(newweights(1,2),maxFrontWeight); max(newweights(2,1),0),min(newweights(2,2),maxRearWeight)];    % if the car is flipping, it can't get more weight
    newweights = newweights+downforce;
    forces=R25Bcornering(newweights)*newweights;
    accel2=sum(forces(:))/w;
    v=v+.1;
    if accel>accel2
        finish=true;
        v=v-.1;
        time=path/v;
        g=((v^2)/r)/9.80665;
%         disp(time)
%         disp(accel)
    end
    i=i+1;
end
    
    
    
