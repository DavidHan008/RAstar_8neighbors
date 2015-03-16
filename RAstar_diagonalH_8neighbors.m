% From pseudocode: http://en.wikipedia.org/wiki/A*_search_algorithm

function [map_with_path,path,g_score,iter,cost,fail,t] = RAstar_diagonalH_8neighbors(isWithTieBreak,map,Nb_iter_max,i_start,j_start,i_goal,j_goal)

%i_start=950;
%j_start=350;
%i_goal=350;
%j_goal=980;
%Nb_iter_max=2000;
path_color=255;
obstacle=100;
M=double(map);
lambda=10;
length=size(M,1);
width=size(M,2);
iter=0;
cost=0;
lateral_cost=1;
diagonal_cost=sqrt(2);
%figure;hold on
i_current=i_start;
j_current=j_start;
%path=[];
map_with_path=map;
fail=0;
if isWithTieBreak, tBreak=1+1/(length+width); else tBreak=1;
Indj=Inf;
tic

sub2ind1 = @(u)(u(:,2)-1)*length+u(:,1);
ind2sub1 = @(k)[rem(k-1, length)+1; (k - rem(k-1, length) - 1)/length + 1];

IndStart=sub2ind1([i_start,j_start]);
Iopen = [IndStart];
%Iclosed=[];

g_score = zeros(length,width)+Inf;
f_score = zeros(length,width)+Inf;
%came_from=zeros(length,width);
g_score(IndStart) = 0;
f_score(IndStart) = diagonal_dist(i_start,j_start,i_goal,j_goal,lateral_cost,diagonal_cost);

current=IndStart;
IndGoal=sub2ind1([i_goal,j_goal]);

while (g_score(IndGoal)==Inf & iter<Nb_iter_max & ~isempty(Iopen)) 
    %[tmp,j] = sort(D(sub2ind1(I))); j = j(1);
    [tmp,j] = min(f_score(Iopen));
    current = Iopen(j); 
    Iopen(j) = [];
 %   Iclosed=[Iclosed;current];
    
    vcurrent=(ind2sub1(current))';
    %Retrieve the list of the neighbors.
    J = get_free_neighbours(vcurrent(1),vcurrent(2), map, obstacle);
    mapv=map(sub2ind1(J));
    % J = get_free_neighbours(i(1),i(2), map, obstacle);
    %Remove those that are dead (no need to consider them anymore).
    %J(S(sub2ind1(J))==-1,:) = [];

%Update neighbor values. For each neightbor  of , perform the update, assuming the length of the edge between  and  is .

for j=J'  
    Indj=sub2ind1(j');

    if g_score(Indj)==Inf
       d0=sqrt((vcurrent(:,1)-j(1)).^2+(vcurrent(:,2)-j(2)).^2) ;
       g_score(Indj)=g_score(current) + d0;
       f_score(Indj)=g_score(Indj)+tBreak*diagonal_dist(j(1),j(2),i_goal,j_goal,lateral_cost,diagonal_cost);
       %if ~(sum(ismember(Iopen,Indj)))
       Iopen=[Iopen;Indj];
       %end
    end
end
       
   
iter=iter+1;
%if ~mod(iter,5000), 
 %   iter, toc, 
  %  size(I,1) 
   %figure(1),imagesc(D), hold on, 
%end

end

if g_score(IndGoal)~=Inf
    disp('***********Path construction:**************')
    i_current=i_goal;
    j_current=j_goal;
    i_start;
    j_start;
    path=[i_goal,j_goal];
    cost=0;
    while (i_current~=i_start | j_current~=j_start) 
        [neighbour_values, index_neighbours] = get_nonVisited_neighbour_values(i_current,j_current, g_score,path); 
         if ~isempty(index_neighbours),
        [m,index_successor]=min(neighbour_values'+sqrt((index_neighbours(:,1)-i_current).^2+(index_neighbours(:,2)-j_current).^2) );
        i_new=index_neighbours(index_successor,1);
        j_new=index_neighbours(index_successor,2);
        if ((i_current==i_new) | (j_current==j_new)), cost=cost+lateral_cost; else cost=cost+diagonal_cost; end
        i_current=i_new;
        j_current=j_new;
        % Addition to path:
        path=[i_current, j_current; path];
        % Addition to the map:
        map_with_path(i_current, j_current)=path_color;
        else deadlock=1, end
    end
    
else disp('Path not found'), path=[], fail=1,
end
iter

t=toc
end
