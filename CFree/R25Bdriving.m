function mu=R25Bdriving(load)
[r,c]=size(load);
mu=zeros(r,c);
mask= load<=50;
mu(mask)=2.624;
mask=load>50 & load<=150;
mu(mask)=2.624+(2.176-2.624)*((load(mask)-50)/(100));
mask=load>150 & load<=200;
mu(mask)=2.176+(2.032-2.176)*((load(mask)-150)/(50));
mask= load>200 & load<=250;
mu(mask)=2.032+(1.976-2.032)*((load(mask)-200)/(50));
mask=load>250;
slope=(1.976-2.032)/(50);
mu(mask)=slope*(load(mask)-250)+1.976;
mu(mu<0)=0;
end