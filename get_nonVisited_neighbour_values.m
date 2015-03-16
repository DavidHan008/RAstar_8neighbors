function [neighbours_values, index_neighbours] = get_nonVisited_neighbour_values(i_current,j_current, M,path)

length=size(M,1);
width=size(M,2);

index_neighbours = get_neighbours(i_current,j_current, length, width);

index_neighbours = setdiff(index_neighbours,path,'rows');

neighbours_values=[];
for k=1:size(index_neighbours,1)
    neighbours_values=[neighbours_values M(index_neighbours(k,1),index_neighbours(k,2))];   
end


end

