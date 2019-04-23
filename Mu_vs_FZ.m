% script to get max grip v. normal load curve
clc, close
mu=NFX*.8;
loads=[-50 -150 -200 -250];
maxes=zeros(1,4);
for i=1:4
    mask=FZ<(loads(i)+1)&FZ>(loads(i)-1);
    grips=abs(mu(mask));
    high=max(grips);
    maxes(i)=high;
end
plot(loads,maxes,'r','linewidth',1)
hold on
plot(FZ,mu,'k.')
curve=polyfit(loads,maxes,5);
otherCurve=pchip(loads,maxes);
x=linspace(-50,-350);
grid on
% plot(x,polyval(curve,x),'b','linewidth',1)
% plot(x,ppval(otherCurve,x),'linewidth',1)
    