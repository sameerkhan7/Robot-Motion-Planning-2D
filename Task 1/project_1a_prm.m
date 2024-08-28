function [V, E, path] = project_1a_prm(lengths, xmax, ymax, B, qi, qg, num_vertices, K)

    %%%% PROJECT TASK 1a: Planar Serial Manipulator (PRM) %%%%

    n = length(lengths);
    
    % Plot workspace
    x = -xmax:0.1:xmax;
    y = -ymax:0.1:ymax;
    [X,Y] = meshgrid(x,y);
    figure;
    plot(X,Y, 'Color', [0.8 0.8 0.8]);
    hold on;
    for i = 1:length(B)
        fill(B{i}(1,:), B{i}(2,:), 'b', 'FaceAlpha', 0.3); 
    end
    rectangle('Position', [-xmax, -ymax, 2*xmax, 2*ymax], 'EdgeColor', 'k', 'LineWidth', 2);
    plot(0, 0, 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
    title('Planar Serial Manipulator: PRM', 'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    % Plot robot in configuration qi
    current_start = [0; 0];
    prior_q = 0;
    q = qi;
    for i=1:n
        vector = [lengths(i)*cos(q(i)+prior_q); lengths(i)*sin(q(i)+prior_q)];
        current_end = current_start + vector;
        plotqi = plot([current_start(1) current_end(1)], [current_start(2) current_end(2)], 'k-');
        current_start = current_end;
        prior_q = prior_q + q(i);
    end
    plot_init = plot(current_end(1), current_end(2), 'mo', 'MarkerSize', 5, 'MarkerFaceColor', 'm');
    
    % Plot robot in configuration qg
    q = qg;
    current_start = [0; 0];
    prior_q = 0;
    for i=1:n
        vector = [lengths(i)*cos(q(i)+prior_q); lengths(i)*sin(q(i)+prior_q)];
        current_end = current_start + vector;
        plot([current_start(1) current_end(1)], [current_start(2) current_end(2)], 'k-');
        current_start = current_end;
        prior_q = prior_q + q(i);
    end
    plot_goal = plot(current_end(1), current_end(2), 'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g');
    
    annotation('textbox', [.31 .77 0.1 0.1], 'String', 'Initial', 'EdgeColor', 'none');
    annotation('textbox', [.725 .12 0.1 0.1], 'String', 'Goal', 'EdgeColor', 'none');
    
    V = {};
    E = {};
    V{1} = qi;
    V{2} = qg;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Create random q's, discard q's that collide
    while width(V) < num_vertices
        % Pick qrand in C (4-dim torus)
        qrand = [rand*2*pi-pi; rand*2*pi-pi; rand*2*pi-pi; rand*2*pi-pi];   % range -pi,pi
        % Plot tentative qrand in yellow
        q = qrand;
        start1 = [0;0];
        end1 = start1 + [lengths(1)*cos(q(1)); lengths(1)*sin(q(1))];
        plot_qrand1 = plot([start1(1) end1(1)], [start1(2) end1(2)], 'y-');
        start2 = end1;
        end2 = start2 + [lengths(2)*cos(q(2)+q(1)); lengths(2)*sin(q(2)+q(1))];
        plot_qrand2 = plot([start2(1) end2(1)], [start2(2) end2(2)], 'y-');
        start3 = end2;
        end3 = start3 + [lengths(3)*cos(q(3)+q(2)+q(1)); lengths(3)*sin(q(3)+q(2)+q(1))];
        plot_qrand3 = plot([start3(1) end3(1)], [start3(2) end3(2)], 'y-');
        start4 = end3;
        end4 = start4 + [lengths(4)*cos(q(4)+q(3)+q(2)+q(1)); lengths(4)*sin(q(4)+q(3)+q(2)+q(1))];
        plot_qrand4 = plot([start4(1) end4(1)], [start4(2) end4(2)], 'y-');
        pause(0.01);
    
        % Check if qrand collides with obstacles. 
        collision = 0;
        S1 = [start1 end1]; S2 = [start2 end2]; S3 = [start3 end3]; S4 = [start4 end4];
        for i = 1:length(B)     % For some obstacle
            Q = make_ccw(B{i});
            b1 = isintersect_linepolygon(S1, Q);
            b2 = isintersect_linepolygon(S2, Q);
            b3 = isintersect_linepolygon(S3, Q);
            b4 = isintersect_linepolygon(S4, Q);
            if b1==1 || b2==1 || b3==1 || b4==1
                collision = 1;
            end
        end
        if collision == 1       % Discard if qrand collides.
            delete(plot_qrand1); delete(plot_qrand2); delete(plot_qrand3); delete(plot_qrand4);
        elseif collision == 0   % Append to V if qrand is collision-free.
            V{end+1} = qrand;
            % Replot in red
            q = qrand;
            start1 = [0;0];
            end1 = start1 + [lengths(1)*cos(q(1)); lengths(1)*sin(q(1))];
            plot_qrand1 = plot([start1(1) end1(1)], [start1(2) end1(2)], 'r-');
            start2 = end1;
            end2 = start2 + [lengths(2)*cos(q(2)+q(1)); lengths(2)*sin(q(2)+q(1))];
            plot_qrand2 = plot([start2(1) end2(1)], [start2(2) end2(2)], 'r-');
            start3 = end2;
            end3 = start3 + [lengths(3)*cos(q(3)+q(2)+q(1)); lengths(3)*sin(q(3)+q(2)+q(1))];
            plot_qrand3 = plot([start3(1) end3(1)], [start3(2) end3(2)], 'r-');
            start4 = end3;
            end4 = start4 + [lengths(4)*cos(q(4)+q(3)+q(2)+q(1)); lengths(4)*sin(q(4)+q(3)+q(2)+q(1))];
            plot_qrand4 = plot([start4(1) end4(1)], [start4(2) end4(2)], 'r-');
            pause(0.01);
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % For each q in V, find K closest neighbors in the C-space, create edges
    for i = 1:width(V) 
        q = V{i};    
        V_all_others = [V(:,1:i-1), V(:,i+1:end)];
    
        % Calculate distances from q to all others
        dist = [];  
        for j = 1:width(V_all_others)                       
            q_prime = V_all_others{j};
            dist(j) = sqrt((q(1)-q_prime(1))^2 + (q(2)-q_prime(2))^2 + (q(3)-q_prime(3))^2 + (q(4)-q_prime(4))^2);
        end
        % Sort all other q's by shortest distance
        [~, others_sorted_idxs] = sort(dist);       % idxs in V, ordered by distance
        K_closest_idxs = others_sorted_idxs(1:K);   % first K of those ordered idxs
        K_closest_neibs = {};
        for k = 1:K
            K_closest_neibs{k} = V_all_others{K_closest_idxs(k)};
        end
       
        % For all K neighbors of some q
        for k=1:K
            neib = K_closest_neibs{k};      % neib is a vector of 4 angles
    
            % Check edge collision between q and neib: 50 increments
            collision = 0;
            for t = 0:0.02:1
                q_inc = [q(1) + t*(neib(1)-q(1));
                         q(2) + t*(neib(2)-q(2));
                         q(3) + t*(neib(3)-q(3));
                         q(4) + t*(neib(4)-q(4))];
                start1 = [0;0];
                end1 = start1 + [lengths(1)*cos(q_inc(1)); lengths(1)*sin(q_inc(1))];
                start2 = end1;
                end2 = start2 + [lengths(2)*cos(q_inc(2)+q_inc(1)); lengths(2)*sin(q_inc(2)+q_inc(1))];
                start3 = end2;
                end3 = start3 + [lengths(3)*cos(q_inc(3)+q_inc(2)+q_inc(1)); lengths(3)*sin(q_inc(3)+q_inc(2)+q_inc(1))];
                start4 = end3;
                end4 = start4 + [lengths(4)*cos(q_inc(4)+q_inc(3)+q_inc(2)+q_inc(1)); lengths(4)*sin(q_inc(4)+q_inc(3)+q_inc(2)+q_inc(1))];
                S1 = [start1 end1]; S2 = [start2 end2]; S3 = [start3 end3]; S4 = [start4 end4];
                for o = 1:length(B)     % For some obstacle
                    Q = make_ccw(B{o});
                    b1 = isintersect_linepolygon(S1, Q);
                    b2 = isintersect_linepolygon(S2, Q);
                    b3 = isintersect_linepolygon(S3, Q);
                    b4 = isintersect_linepolygon(S4, Q);
                    if b1==1 || b2==1 || b3==1 || b4==1
                        collision = 1;
                    end
                end
            end
    
            % Check if an edge between q and neib already exists
            already_edge = 0;
            for e = 1:length(E)
                if isequal([neib, q], E{e})
                    already_edge = 1;
                elseif isequal([q, neib], E{e})
                    already_edge = 1;
                end
            end
    
            % Check that edge length is not zero
            distance = sqrt((q(1)-neib(1))^2 + (q(2)-neib(2))^2 + (q(3)-neib(3))^2 + (q(4)-neib(4))^2);
            if already_edge == 0 && distance > 0 && collision == 0
                E{end+1} = [q, neib];
            end
    
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Now we have complete V and E.  Assemble weighted adjacency matrix G:
    G = zeros(width(V));
    G(~eye(width(V))) = inf;
    for i = 1:length(E)
        edge_vertices = E{i};                                       % Go through all edges [q, neib]
        % Find index of edge_vertices(:,1) in V: First side of the edge (q) is located where in V?
        for j = 1:length(V)
            if isequal(edge_vertices(:,1), V{j})
                idx1 = j;
                break;
            end
        end
    
        % Find index of edge_vertices(:,2) in V: Second side of the edge (neib) is located where in V?
        for j = 1:length(V)
            if isequal(edge_vertices(:,2), V{j})
                idx2 = j;
                break;
            end
        end
    
        if idx1 > 0 && idx2 > 0
            % Weight = distance along edge
            weight = norm(V{idx1} - V{idx2});
            G(idx1, idx2) = weight;
            G(idx2, idx1) = weight;
        end
    
    end
    
    % Construct path with Dijkstra algorithm: qi = V{1}, qg = V{2}
    [~, shortest_path] = dijkstra(G, 1, 2);
    
    % The shortest_path returns idx's that correspond with q's in V
    path = [];
    for i = 1:length(shortest_path)
        path = [path, V{shortest_path(i)}];     % Matrix 'path' is a series of q vectors
    end
    
    
    % Graph the shortest path configurations in blue:
    for i = 1:length(path)
        q = path(:, i);
        current_start = [0; 0];
        prior_q = 0;
        for p = 1:4
            vector = [lengths(p)*cos(q(p)+prior_q); lengths(p)*sin(q(p)+prior_q)];
            current_end = current_start + vector;
            plot([current_start(1) current_end(1)], [current_start(2) current_end(2)], 'Color', [0, 0, 1]);
            current_start = current_end;
            prior_q = prior_q + q(p);
        end
    end
    pause(0.25);
        
    
    % Illustrate the trajectory of the robot in rainbow: 
    rainbow = colormap(hsv( width(path) + (width(path)-1)*50 ));
    rainbow_row = 1;
    for i = 1:length(path)-1
        q1 = path(:, i);
        q2 = path(:, i+1);
        
           for t = 0:0.02:1    % 50 increments between each 2 configurations
                q_inc = [q1(1) + t*(q2(1)-q1(1));
                         q1(2) + t*(q2(2)-q1(2));
                         q1(3) + t*(q2(3)-q1(3));
                         q1(4) + t*(q2(4)-q1(4))];
                start1 = [0;0];
                end1 = start1 + [lengths(1)*cos(q_inc(1)); lengths(1)*sin(q_inc(1))];
                start2 = end1;
                end2 = start2 + [lengths(2)*cos(q_inc(2)+q_inc(1)); lengths(2)*sin(q_inc(2)+q_inc(1))];
                start3 = end2;
                end3 = start3 + [lengths(3)*cos(q_inc(3)+q_inc(2)+q_inc(1)); lengths(3)*sin(q_inc(3)+q_inc(2)+q_inc(1))];
                start4 = end3;
                end4 = start4 + [lengths(4)*cos(q_inc(4)+q_inc(3)+q_inc(2)+q_inc(1)); lengths(4)*sin(q_inc(4)+q_inc(3)+q_inc(2)+q_inc(1))];
    
                color = rainbow(rainbow_row, :);
                plot([start1(1) end1(1)], [start1(2) end1(2)], 'Color', color);     % these 4 commands all plotted in the same rainbow(i,:)
                plot([start2(1) end2(1)], [start2(2) end2(2)], 'Color', color);
                plot([start3(1) end3(1)], [start3(2) end3(2)], 'Color', color);
                plot([start4(1) end4(1)], [start4(2) end4(2)], 'Color', color);
                % Plot in thick black:
                bk1 = plot([start1(1) end1(1)], [start1(2) end1(2)], 'Color', 'k', 'LineWidth', 2);
                bk2 = plot([start2(1) end2(1)], [start2(2) end2(2)], 'Color', 'k', 'LineWidth', 2);
                bk3 = plot([start3(1) end3(1)], [start3(2) end3(2)], 'Color', 'k', 'LineWidth', 2);
                bk4 = plot([start4(1) end4(1)], [start4(2) end4(2)], 'Color', 'k', 'LineWidth', 2);
                pause(0.003);
                % delete the black
                delete(bk1); delete(bk2); delete(bk3); delete(bk4);
                rainbow_row = rainbow_row + 1;
           end
    end
    
    bk1 = plot([start1(1) end1(1)], [start1(2) end1(2)], 'Color', 'k', 'LineWidth', 2);
    bk2 = plot([start2(1) end2(1)], [start2(2) end2(2)], 'Color', 'k', 'LineWidth', 2);
    bk3 = plot([start3(1) end3(1)], [start3(2) end3(2)], 'Color', 'k', 'LineWidth', 2);
    bk4 = plot([start4(1) end4(1)], [start4(2) end4(2)], 'Color', 'k', 'LineWidth', 2);
    
end
    

