function [V, E, path] = project_4(lengths, xmax, ymax, zmax, yobs, xobs, qi, qg, dq, near_qg_thresh)
    
    % Task 4: RRT for 3D serial manipulator (UR5)
   
    % Plot workspace
    x = [-xmax, xmax];
    y = [-ymax, ymax];
    z = [-zmax, zmax];
    [X, Y, Z] = meshgrid(x(1):10:x(2), y(1):10:y(2), z(1):10:z(2));
    figure;
    scatter3(x(:), y(:), z(:), 1, 'b');
    axis equal;
    grid on;
    hold on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('UR5 Robot: RRT', 'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    set(gcf, 'Position', [100 100 800 600]);
    pause(0.5);
    
    % Plot obstacles
    % Obstacle 1:  plane at y = yobs
    [x_plane, z_plane] = meshgrid(x,z);
    y_plane = 200 * ones(size(x_plane));
    surf(x_plane, y_plane, z_plane, 'FaceColor', 'cyan', 'FaceAlpha', 0.2);
    % Obstacle 2:  plane at x = xobs
    [y_plane, z_plane] = meshgrid(y,z);
    x_plane = 200 * ones(size(y_plane));
    surf(x_plane, y_plane, z_plane, 'FaceColor', 'magenta', 'FaceAlpha', 0.2);
    pause(0.5);
    
    % Plot robot
    plot_config(lengths, qi, 'r', 'ro');    % q is 8x1
    plot_config(lengths, qg, 'g', 'go');
    pause(0.5);
    
    V = {};
    E = {};
    V{1} = qi;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Repeat until our qnew is "near" qg (breaks out)
    near_qg = 0;
    while near_qg == 0
        % Get some qrand in C-space. 
        % (EVERY 2 NEW CONFIGS, USE qg AS qrand)
        if mod(size(V), 2) == 0
            qrand = qg;
        else
            qrand = [rand*2*pi-pi; rand*2*pi-pi; rand*2*pi-pi; rand*2*pi-pi;
                rand*2*pi-pi; rand*2*pi-pi; rand*2*pi-pi];   % 7 angles, range -pi,pi
        end
    
        % Find the q in V that is nearest to qrand.
        qnear = nearest_vertex_ur5(qrand, V);
    
        % Select qnew by moving from qnear by the step size dq, in direction toward qrand
        qdir = qrand - qnear;
        qnew = qnear + dq*(qdir/norm(qdir));
    
        % Plot tentative qnew in yellow
        q = qnew;
        plot_config(lengths, qnew, 'y', 'yo');
    
        % Check collision of trajectory from qnear to qnew (implies qnew collision)
        collision = 0;
        for t = 0:0.02:1
            q_inc = [qnear(1) + t*(qnew(1)-qnear(1));
                     qnear(2) + t*(qnew(2)-qnear(2));
                     qnear(3) + t*(qnew(3)-qnear(3));
                     qnear(4) + t*(qnew(4)-qnear(4));
                     qnear(5) + t*(qnew(5)-qnear(5));
                     qnear(6) + t*(qnew(6)-qnear(6));
                     qnear(7) + t*(qnew(7)-qnear(7))];
            vertices_inc = get_vertices(lengths, q_inc);    % 9 columns of homogeneous 3D coords
            % In this incremental configuration, check if each vertex crosses the planes (y=200, x=200):
            for k = 1:length(vertices_inc)
                v = vertices_inc(1:3, k);
                if v(1) >= 200 || v(2) >= 200
                    collision = 1;
                end
            end
        end
        % If edge is collision-free, append qnew to V, append edge to E 
        if collision == 0   
            V{end+1} = qnew;
            E{end+1} = [qnear, qnew];
            % Replot qnew in blue
            plot_config(lengths, qnew, 'b', 'bo');
            pause(0.0005);
        elseif collision == 1
            % Delete plots of qnew
            % Not sure how
            a = 2;
        end
    
        % Check if qnew end vertex is "near" qg end vertex
        vertices_qnew = get_vertices(lengths, qnew);
        vertices_qg = get_vertices(lengths, qg);
        end_qnew = vertices_qnew(1:3, 7);
        end_qg = vertices_qg(1:3, 7);
        if end_qnew(1) >= end_qg(1)-200 && end_qnew(1) <= end_qg(1)+200
            if end_qnew(2) >= end_qg(2)-200 && end_qnew(2) <= end_qg(2)+200
                if end_qnew(3) >= end_qg(3)-200 && end_qnew(3) <= end_qg(3)+200
        % dist_qnew_qg = norm(qnew-qg);
        % if dist_qnew_qg <= near_qg_thresh
                    % Check collision along the edge from qnew to qg
                    collision = 0;
                    for t = 0:0.02:1
                        q_inc = [qg(1) + t*(qnew(1)-qg(1));
                                 qg(2) + t*(qnew(2)-qg(2));
                                 qg(3) + t*(qnew(3)-qg(3));
                                 qg(4) + t*(qnew(4)-qg(4));
                                 qg(5) + t*(qnew(5)-qg(5));
                                 qg(6) + t*(qnew(6)-qg(6));
                                 qg(7) + t*(qnew(7)-qg(7))];
                        vertices_inc = get_vertices(lengths, q_inc);    % 9 columns of homogeneous 3D coords
                        % In this incremental configuration, check if each vertex crosses the planes (y=200, x=200)
                        for k = 1:length(vertices_inc)
                            v = vertices_inc(1:3, k);
                            if v(1) >= 200 || v(2) >= 200
                                collision = 1;
                            end
                        end
                    end
                    % If qnew reaches qg: Add edge [qnew qg] to E, break out
                    if collision == 0
                        E{end+1} = [qnew, qg];
                        near_qg = 1;    % this breaks out
                    end
                end
            end
        end
    end % while near_qg == 0
    
    
    pause(0.25);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Now that we have all vertices,
    last_edge = E{end};
    if last_edge(:,2) == qg         % If we can reach qg
        path = qg;
        curr_q = qg;
        complete = 0;
        parent = [];
        while complete == 0
            % Search E for the edge [parent, curr_q]
            for i=1:length(E)
                e = E{i};
                if e(1,2)==curr_q(1) && e(2,2)==curr_q(2) && e(3,2)==curr_q(3) && e(4,2)==curr_q(4) && e(5,2)==curr_q(5) && e(6,2)==curr_q(6) && e(7,2)==curr_q(7)
                    parent = e(:,1);
                end
            end
    
            % Add parent to the front of path
            path = [parent, path];  
            curr_q = parent;
    
            % If path contains qi, complete
            if path(:,1) == qi
                complete = 1;
            end
        end
    
        % Graph the path in cyan:
        for i = 2:width(path)-1
            q = path(:, i);
            plot_config(lengths, q, 'c', 'co');
        end
        pause(0.25);
    
        % Illustrate the trajectory of the robot in rainbow:
        rainbow = colormap(hsv( width(path) + (width(path)-1)*25 ));
        rainbow_row = 1;
        for i = 1:width(path)-1
            config1 = path(:, i);
            config2 = path(:, i+1);
            
               for t = 0:0.04:1    % 25 increments between each 2 configurations
                    q_inc = [config1(1) + t*(config2(1)-config1(1));
                             config1(2) + t*(config2(2)-config1(2));
                             config1(3) + t*(config2(3)-config1(3));
                             config1(4) + t*(config2(4)-config1(4));
                             config1(5) + t*(config2(5)-config1(5));
                             config1(6) + t*(config2(6)-config1(6));
                             config1(7) + t*(config2(7)-config1(7))];
                    color = rainbow(rainbow_row, :);
                    vertices = get_vertices(lengths, q_inc);
    
                    v1 = vertices(:,1);
                    v2 = vertices(:,2);
                    v3 = vertices(:,3);
                    v4 = vertices(:,4);
                    v5 = vertices(:,5);
                    v6 = vertices(:,6);
                    v7 = vertices(:,7);
                    v8 = vertices(:,8);
                    v9 = vertices(:,9);
                    plot_v1 = plot3(v1(1), v1(2), v1(3), 'MarkerSize', 2, 'MarkerFaceColor', color);
                    plot_v2 = plot3(v2(1), v2(2), v2(3), 'MarkerSize', 2, 'MarkerFaceColor', color);
                    plot_v3 = plot3(v3(1), v3(2), v3(3), 'MarkerSize', 2, 'MarkerFaceColor', color);
                    plot_v4 = plot3(v4(1), v4(2), v4(3), 'MarkerSize', 2, 'MarkerFaceColor', color);
                    plot_v5 = plot3(v5(1), v5(2), v5(3), 'MarkerSize', 2, 'MarkerFaceColor', color);
                    plot_v6 = plot3(v6(1), v6(2), v6(3), 'MarkerSize', 2, 'MarkerFaceColor', color);
                    plot_v7 = plot3(v7(1), v7(2), v7(3), 'MarkerSize', 2, 'MarkerFaceColor', color);
                    plot_v8 = plot3(v8(1), v8(2), v8(3), 'MarkerSize', 2, 'MarkerFaceColor', color);
                    plot_v9 = plot3(v9(1), v9(2), v9(3), 'MarkerSize', 2, 'MarkerFaceColor', color);
                    plot_l1 = plot3([v1(1) v2(1)], [v1(2) v2(2)], [v1(3) v2(3)], 'Color', color);
                    plot_l2 = plot3([v2(1) v3(1)], [v2(2) v3(2)], [v2(3) v3(3)], 'Color', color);
                    plot_l3 = plot3([v3(1) v4(1)], [v3(2) v4(2)], [v3(3) v4(3)], 'Color', color);
                    plot_l4 = plot3([v4(1) v5(1)], [v4(2) v5(2)], [v4(3) v5(3)], 'Color', color);
                    plot_l5 = plot3([v5(1) v6(1)], [v5(2) v6(2)], [v5(3) v6(3)], 'Color', color);
                    plot_l6 = plot3([v6(1) v7(1)], [v6(2) v7(2)], [v6(3) v7(3)], 'Color', color);
                    plot_l7 = plot3([v7(1) v8(1)], [v7(2) v8(2)], [v7(3) v8(3)], 'Color', color);
                    plot_l8 = plot3([v8(1) v9(1)], [v8(2) v9(2)], [v8(3) v9(3)], 'Color', color);
                    plot_v1k = plot3(v1(1), v1(2), v1(3), 'MarkerSize', 4, 'MarkerFaceColor', 'k');
                    plot_v2k = plot3(v2(1), v2(2), v2(3), 'MarkerSize', 4, 'MarkerFaceColor', 'k');
                    plot_v3k = plot3(v3(1), v3(2), v3(3), 'MarkerSize', 4, 'MarkerFaceColor', 'k');
                    plot_v4k = plot3(v4(1), v4(2), v4(3), 'MarkerSize', 4, 'MarkerFaceColor', 'k');
                    plot_v5k = plot3(v5(1), v5(2), v5(3), 'MarkerSize', 4, 'MarkerFaceColor', 'k');
                    plot_v6k = plot3(v6(1), v6(2), v6(3), 'MarkerSize', 4, 'MarkerFaceColor', 'k');
                    plot_v7k = plot3(v7(1), v7(2), v7(3), 'MarkerSize', 4, 'MarkerFaceColor', 'k');
                    plot_v8k = plot3(v8(1), v8(2), v8(3), 'MarkerSize', 4, 'MarkerFaceColor', 'k');
                    plot_v9k = plot3(v9(1), v9(2), v9(3), 'MarkerSize', 4, 'MarkerFaceColor', 'k');
                    plot_l1k = plot3([v1(1) v2(1)], [v1(2) v2(2)], [v1(3) v2(3)], 'Color', 'k', 'LineWidth', 4);
                    plot_l2k = plot3([v2(1) v3(1)], [v2(2) v3(2)], [v2(3) v3(3)], 'Color', 'k', 'LineWidth', 4);
                    plot_l3k = plot3([v3(1) v4(1)], [v3(2) v4(2)], [v3(3) v4(3)], 'Color', 'k', 'LineWidth', 4);
                    plot_l4k = plot3([v4(1) v5(1)], [v4(2) v5(2)], [v4(3) v5(3)], 'Color', 'k', 'LineWidth', 4);
                    plot_l5k = plot3([v5(1) v6(1)], [v5(2) v6(2)], [v5(3) v6(3)], 'Color', 'k', 'LineWidth', 4);
                    plot_l6k = plot3([v6(1) v7(1)], [v6(2) v7(2)], [v6(3) v7(3)], 'Color', 'k', 'LineWidth', 4);
                    plot_l7k = plot3([v7(1) v8(1)], [v7(2) v8(2)], [v7(3) v8(3)], 'Color', 'k', 'LineWidth', 4);
                    plot_l8k = plot3([v8(1) v9(1)], [v8(2) v9(2)], [v8(3) v9(3)], 'Color', 'k', 'LineWidth', 4);
                    pause(0.003);
                    delete(plot_v1k); delete(plot_v2k); delete(plot_v3k); delete(plot_v4k);
                    delete(plot_v5k); delete(plot_v6k); delete(plot_v7k); delete(plot_v8k);
                    delete(plot_v9k);
                    delete(plot_l1k); delete(plot_l2k); delete(plot_l3k); delete(plot_l4k); 
                    delete(plot_l5k); delete(plot_l6k); delete(plot_l7k); delete(plot_l8k);
                    rainbow_row = rainbow_row + 1;
               end
        end
        
    
    else
        % IF FAILURE: reached max num_nodes. return path = 0
        path = 0;
    end
    
    
end 

