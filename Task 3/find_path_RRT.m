function [path] = find_path_RRT(idx_I,idx_G,V,E)
%find path from q_I to q_G using G = [V,E] using there idx
path = [];
%found q_I
if idx_G == 1
    path = [path,V(:,idx_I)];
    return
end

path = find_path_RRT(idx_I, E(idx_G), V, E);
path = [path,V(:,idx_G)];

end

