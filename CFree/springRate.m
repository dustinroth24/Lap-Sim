% spring rate script
clc
mask=P>=12 & P<13;
plot(9-RL(mask),abs(FZ(mask)),'k.')
x=9-RL(mask);
k=mean(abs(FZ(mask))./x)
hold on
plot(x,k*x,'r')
% k=335 lbf/in