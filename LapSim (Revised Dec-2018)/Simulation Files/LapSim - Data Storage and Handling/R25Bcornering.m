function mu = R25Bcornering(load,commie)
%% FUNCTION: R25Bcornering
%
% Author:        Christian Free
% Modified:      Dec. 19 2018
%
% Purpose:       Calculate mu of R25B under a load in lateral accel
%
% Misc. Notes:   1) A more accurate tire model could be a good addition to
%                   the sim, but at that point a commercial sim package
%                   would probably be better.
%
% Inputs:        1) load: vector containing corner normal loads (lb or N)
%                2) commie: boolean of whether inputs are lb or N.  true
%                   for N, false for lb. If no second input, then false
%
% Outputs:       1) mu: vector of friction coefficients

%% See Number of Inputs
nInputs = nargin;   % find number of inputs
switch nInputs
    case 1
        commie = false; % default value for commie is false
    case 2
end

%% Compute Friction by Constructing Maps
if commie
    [r,c]=size(load);
    mu=zeros(r,c);
    mu(load<=222.4)=2.328;
    mu(load<0)=0;
    mask=(load>222.4 & load<=444.822);
        mu(mask) = myInterpolate(load(mask),222.4,444.8,2.328,2.192);
    mask=load>444.8 & load<=667.2;
        mu(mask) = myInterpolate(load(mask),444.8,667.2,2.192,2.176);
    mask=load>667.2 & load<=1112.1;
        mu(mask) = myInterpolate(load(mask),667.2,1112.1,2.176,2.232);
    mask=load>1112.1 & load<=1556.9;
        mu(mask) = myInterpolate(load(mask),1112.1,1556.9,2.232,1.696);
    mask=load>1556.9;
        slope=(1.696-2.232)/(1556.9-1112.1);
        mu(mask)=slope*(load(mask)-1556.9)+1.696;
    mu(mu<0)=0;
else   % assume not communist
    [r,c]=size(load);
    mu=zeros(r,c);
    mu(load<=50)=2.328;
    mu(load<0)=0;
    mask=(load>50 & load<=100);
        mu(mask) = myInterpolate(load(mask),50,100,2.328,2.192);
    mask=load>100 & load<=150;
        mu(mask) = myInterpolate(load(mask),100,150,2.192,2.176);
    mask=load>150 & load<=250;
        mu(mask) = myInterpolate(load(mask),150,250,2.176,2.232);
    mask=load>250 & load<=350;
        mu(mask) = myInterpolate(load(mask),250,350,2.232,1.696);
    slope=(1.696-2.232)/(100);
    mask=load>350;
        slope=(1.696-2.232)/(100);
        mu(mask)=slope*(load(mask)-350)+1.696;
    mu(mu<0)=0;
end
end

%% Helper Function to Interpolate
function y=myInterpolate(x,xa,xb,ya,yb)
    y = ya + (yb-ya).*(x-xa)./(xb-xa);
end

    
    
