function [q_near,i_near] = nearest_vertex(q,V)
%find nearest vertex in G to q
d = inf;
for i=1:length(V(1,:)) %loop through all V
    %find smallest dist between q and V(i)
    temp = norm(V(:,i)-q);
    if temp < d
        q_near = V(:,i);
        i_near = i;
        d = temp;
    end
end
end

