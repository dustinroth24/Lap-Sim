function mu=R25Bcornering(load)
[r,c]=size(load);
mu=zeros(r,c);
mu(load<=50)=2.328;
mu(load<0)=0;
mask1=(load>50 & load<=100);
mu(mask1)=2.328+(2.192-2.328)*((load(mask1)-50)/(50));
mask2=load>100 & load<=150;
mu(mask2)=2.192+(2.176-2.192)*((load(mask2)-100)/(50));
mask3=load>150 & load<=250;
mu(mask3)=2.176+(2.232-2.176)*((load(mask3)-150)/(100));
mask4=load>250 & load<=350;
mu(mask4)=2.232+(1.696-2.232)*((load(mask4)-250)/(100));
slope=(1.696-2.232)/(100);
mask5=load>350;
mu(mask5)=slope*(load(mask5)-350)+1.696;
mu(mu<0)=0;
end
    

    
    
