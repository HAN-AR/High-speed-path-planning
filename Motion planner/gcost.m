function gc = gcost(cur,cost1)
% Calculation of the path length of motion primitive. 
%d1       =   hypot(diff(xp),diff(yp));
d1=cost1;
d_tot   =  (d1);   % d_tot= length of motion primitive;

if isempty(cur.gcost) % For the first iteration.

gc      = (0+0+d_tot);
                                       
else                                                                       
gc      =((0+cur.gcost+d_tot));    % For subsequent iterations. 
     
end