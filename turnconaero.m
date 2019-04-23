function [time,v,g]=turnconaero(r,theta,carinfo)
% car info is c, w, b, t, h
% units are m and m/s
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
while finish==false
    accel=(v^2/r)/9.81;
    dw=(w*accel*h/t)/2; % weight transfer
    downforce=polyval(aeromap,v)/4;
%     downforce=0; % for when I want to ignore aero for a sec
    newweights=cornerweights+[-dw dw;-dw dw]+downforce;
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
    
    
    
