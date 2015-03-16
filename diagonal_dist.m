function [dist] = diagonal_dist(i1,j1,i2,j2,lateral_cost,diagonal_cost)

dx=abs(i1-i2);
dy=abs(j1-j2);
if dx<dy,
    dist=dx*diagonal_cost+lateral_cost*(dy-dx);
else
    dist=dy*diagonal_cost+lateral_cost*(dx-dy); 
end

end

