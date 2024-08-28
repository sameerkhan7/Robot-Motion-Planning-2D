function [dist, prev, path] = dijkstra(G,n_init,n_goal)
%initialize arrays
dim = size(G,1);
prev = zeros(1,dim);
dist = inf(1,dim);
%intial values
dist(n_init) = 0;
U = 1:dim; %if in U = unvisited

while ismember(n_goal,U) %while goal node is in U
    [~,ind] = min(dist(U)); %find smallest distance node in U
    curr = U(ind); %set curr to smallest distance node curr = C from pseudocode
    U = U(~ismember(U,curr)); %remove curr from U
    for n = 1:dim %for each node
        if G(curr,n) < inf && (curr ~= n) %only if node is neighbor
            alt = dist(curr) + G(curr,n); %calculate path to nieghbor
            if alt < dist(n) %if new path is shorter than current dist
                dist(n) = alt; %update distance of neighbor
                prev(n) = curr; %update parent of neighbor
            end
        end
    end
end
%find sequential path
path_node = n_goal;
path = n_goal;
while prev(path_node) ~= 0
    path = [path, prev(path_node)];
    path_node = prev(path_node);
end
path = flip(path);
cost = dist(n_goal); % total cost of path
end
