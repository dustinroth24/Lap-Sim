% Script: Estimating highest cornering and longitudinal g's
clear
clc
c=.47; % weight distributiuon %front
w=600; % weight of car
b=60.5; % wheelbase
t=48; % trackwidth
h=14; % cg height off the ground
car=[c w b t h];
cornerweights=[w*c/2 w*c/2;w*(1-c)/2 w*(1-c)/2];
sideforce=cornerweights.*R25Bcornering(cornerweights);
brakeforce=cornerweights.*R25Bdriving(cornerweights);
accelforce=cornerweights(2,:).*R25Bdriving(cornerweights(2,:));
g1=sum(sideforce(:))/w;
gb=sum(brakeforce(:))/w;
ga=sum(accelforce)/w;
et=inf;
while et>.1
    dw=w*h*g1/2/t; % weight transferred (lbs) per axle
    newweights=[cornerweights(1,1)-dw cornerweights(1,2)+dw; cornerweights(2,1)-dw cornerweights(2,2)+dw];
    newsideforce=newweights.*R25Bcornering(newweights);
    g2=sum(newsideforce(:))/w;
    et=abs(g2-g1);
    g1=g2;
end
latg=g1;
%%
et=inf;
g1=gb;
while et>.1
    dw=w*h*g1/2/b;
    newweights=[cornerweights(1,1)+dw cornerweights(1,2)+dw; cornerweights(2,1)-dw cornerweights(2,2)-dw];
    newbrakeforce=newweights.*R25Bdriving(newweights);
    g2=sum(newbrakeforce(:))/w;
    et=abs(g2-g1);
    g1=g2;
end
brakeg=g1;
%%
et=inf;
g1=ga;
while et>.1
    dw=w*h*g1/2/b;
    newweights=[cornerweights(1,1)-dw cornerweights(1,2)-dw; cornerweights(2,1)+dw cornerweights(2,2)+dw];
    newaccelforce=newweights.*R25Bdriving(newweights);
    newaccelforce(1,:)=0;
    g2=sum(newaccelforce(:))/w;
    et=abs(g2-g1);
    g1=g2;
end
accelg=g1;
%%
FSAEM2018_Simp([accelg,brakeg],car);

