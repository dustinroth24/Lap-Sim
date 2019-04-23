function mu=R25Bcornering(load)
x = [50 100 150 250 350];
y = [2.328 2.2 2.2 2.2 1.696];
[r,c]=size(load);
% mu = zeros(r, c);
mask = load >= 50 & load <= 350;
mu = interp1(x, y, load, 'method', 'extrap');
mu(mu<0) = 0;
if any(~mask)
%     mu(load > 350) = 2.328;
%     mu(load < 50) = 1.84;
end

    

    
    
