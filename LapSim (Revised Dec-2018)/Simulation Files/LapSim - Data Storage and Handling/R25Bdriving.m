function mu=R25Bdriving(load,commie)
%% FUNCTION: R25Bcornering
%
% Author:        Christian Free
% Modified:      Dec. 19 2018
%
% Purpose:       Calculate mu of R25B under a load in longitudinal accel
%
% Misc. Notes:   1) A more accurate tire model could be a good addition to
%                   the sim, but at that point a commercial sim package
%                   would probably be better.
%
% Inputs:        1) load: vector containing corner normal loads (lb or N)
%                2) commie: boolean of whether inputs are lb or N.  true
%                   for N, false for lb.  If no second input, then false
%
% Outputs:       1) mu: vector of friction coefficients

nInputs = nargin;   % find number of inputs
switch nInputs
    case 1
        commie = false; % default value for commie is false
    case 2
end
if commie
    [r,c]=size(load);
    mu=zeros(r,c);
    mask= load<=222.4;
    mu(mask)=2.624;
    mask=load>222.4 & load<=667.2;
        mu(mask) = myInterpolate(load(mask),222.4,667.2,2.624,2.176);
    mask=load>667.2 & load<=889.6;
        mu(mask) = myInterpolate(load(mask),667.2,889.6,2.176,2.032); 
    mask= load>889.6 & load<=1112.06;
        mu(mask) = myInterpolate(load(mask),889.6,1112.06,2.032,1.976);
    mask=load>1112.06;
        slope=(1.976-2.032)/(1112.06-889.6);
        mu(mask)=slope*(load(mask)-1112.06)+1.976;
    mu(mu<0)=0;
else   % assume not communist
    [r,c]=size(load);
    mu=zeros(r,c);
    mask= load<=50;
    mu(mask)=2.624;
    mask=load>50 & load<=150;
        mu(mask) = myInterpolate(load(mask),50,150,2.624,2.176);
    mask=load>150 & load<=200;
        mu(mask) = myInterpolate(load(mask),150,200,2.176,2.032);  
    mask= load>200 & load<=250;
        mu(mask) = myInterpolate(load(mask),200,250,2.032,1.976);
    mask=load>250;
        slope=(1.976-2.032)/(50);
        mu(mask)=slope*(load(mask)-250)+1.976;
    mu(mu<0)=0;    
end

end
function y=myInterpolate(x,xa,xb,ya,yb)
    y = ya + (yb-ya).*(x-xa)./(xb-xa);
end