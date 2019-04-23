function [time,score,position]=accelSimp(a)
accel=a*9.81;
time=sqrt(2*75/accel);
[num,~,~]=xlsread('2018_accelResults.xlsx');
Tmin=min(num);
if time<Tmin
    Tmin=time;
end
Tmax=1.5*Tmin;
score=(((Tmax/time)^2)-1)/(((Tmax/Tmin)^2)-1);
score=95.5*score+4.5;
index=time<num;
position=find(index,1);
if score<4.5
    score=4.5;
end
end