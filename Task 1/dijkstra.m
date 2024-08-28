
% Dijkstra's Algorithm

function [lowest_cost, shortest_path] = dijkstra(graph, n_init, n_goal)

    num_nodes = length(graph);
    dist = inf(1, num_nodes);       % All node distances (initialize to inf)
    prev = zeros(1, num_nodes);     % All node parents (initialize to 0)

    dist(n_init) = 0;
    U = 1:num_nodes;                % U: all nodes we still need to visit

    % While goal node is in U:
    while ismember(n_goal, U)
        % C = node in U with smallest distance
        curr_min_node = inf;
        curr_min_val = inf;
        for i = 1:length(U)
            if dist(U(i)) < curr_min_val
                curr_min_val = dist(U(i));
                curr_min_node = U(i);
            end
        end
        C = curr_min_node;

        % Remove node C from U
        U = setdiff(U, C);

        % Get neighbors of node C: Find values in graph column C that are > 0
        neighbors = [];
        for i = 1:length(graph)
            if graph(i, C) > 0
                neighbors = [neighbors, i];
            end
        end

        % For each neighbor (v) of current node (C):
        for i = 1:length(neighbors)
            v = neighbors(i);
            alt = dist(C) + graph(C,v); % Total distance to that neighbor
            if alt < dist(v)            % Update with the minimum total
                dist(v) = alt;
                prev(v) = C;
            end
        end
    end

    % Construct the shortest path:
    shortest_path = [n_goal];
    curr_node = n_goal;
    while curr_node ~= n_init
        shortest_path = [prev(curr_node), shortest_path];
        curr_node = prev(curr_node);
    end

    % Find the lowest cost:
    lowest_cost = 0;
    for i = 1:(length(shortest_path)-1)
        node_1 = shortest_path(i);
        node_2 = shortest_path(i+1);
        lowest_cost = lowest_cost + graph(node_1, node_2);
    end
    
end

