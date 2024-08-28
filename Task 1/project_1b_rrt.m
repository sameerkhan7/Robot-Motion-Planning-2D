function [V, E, path] = project_1b_rrt(lengths, xmax, ymax, B, qi, qg, dq, near_qg_thresh)

    %%%% PROJECT TASK 1b: Planar Serial Manipulator (RRT) %%%%

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
    title('Planar Serial Manipulator: RRT', 'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
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
    plot(current_end(1), current_end(2), 'mo', 'MarkerSize', 5, 'MarkerFaceColor', 'm');
    
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
    plot(current_end(1), current_end(2), 'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g');
    
    annotation('textbox', [.31 .77 0.1 0.1], 'String', 'Initial', 'EdgeColor', 'none');
    annotation('textbox', [.725 .12 0.1 0.1], 'String', 'Goal', 'EdgeColor', 'none');
    
    V = {};
    E = {};
    V{1} = qi;
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Repeat until our qnew is "near" qg (breaks out)
    near_qg = 0;
    while near_qg == 0
        % Get some qrand in C-space
        qrand = [rand*2*pi-pi; rand*2*pi-pi; rand*2*pi-pi; rand*2*pi-pi];   % range -pi,pi
    
        % Find the q in V that is nearest to qrand
        qnear = nearest_vertex_serial(qrand, V);
    
        % Select qnew by moving from qnear by the step size dq, in direction toward qrand
        qdir = qrand - qnear;
        qnew = qnear + dq*(qdir/norm(qdir));
    
        % Plot tentative qnew in yellow
        q = qnew;
        start1 = [0;0];
        end1 = start1 + [lengths(1)*cos(q(1)); lengths(1)*sin(q(1))];
        plot_qnew1 = plot([start1(1) end1(1)], [start1(2) end1(2)], 'y-');
        start2 = end1;
        end2 = start2 + [lengths(2)*cos(q(2)+q(1)); lengths(2)*sin(q(2)+q(1))];
        plot_qnew2 = plot([start2(1) end2(1)], [start2(2) end2(2)], 'y-');
        start3 = end2;
        end3 = start3 + [lengths(3)*cos(q(3)+q(2)+q(1)); lengths(3)*sin(q(3)+q(2)+q(1))];
        plot_qnew3 = plot([start3(1) end3(1)], [start3(2) end3(2)], 'y-');
        start4 = end3;
        end4 = start4 + [lengths(4)*cos(q(4)+q(3)+q(2)+q(1)); lengths(4)*sin(q(4)+q(3)+q(2)+q(1))];
        plot_qnew4 = plot([start4(1) end4(1)], [start4(2) end4(2)], 'y-');
        pause(0.01);
    
        % Check collision of edge from qnear to qnew (implies qnew)
        collision = 0;
        for t = 0:0.02:1
            q_inc = [qnear(1) + t*(qnew(1)-qnear(1));
                     qnear(2) + t*(qnew(2)-qnear(2));
                     qnear(3) + t*(qnew(3)-qnear(3));
                     qnear(4) + t*(qnew(4)-qnear(4))];
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
        % If edge is collision-free, append qnew to V, append edge to E 
        if collision == 0   
            V{end+1} = qnew;
            E{end+1} = [qnear, qnew];
            % Replot qnew in blue
            q = qnew;
            start1 = [0;0];
            end1 = start1 + [lengths(1)*cos(q(1)); lengths(1)*sin(q(1))];
            plot_qnew1 = plot([start1(1) end1(1)], [start1(2) end1(2)], 'r-');
            start2 = end1;
            end2 = start2 + [lengths(2)*cos(q(2)+q(1)); lengths(2)*sin(q(2)+q(1))];
            plot_qnew2 = plot([start2(1) end2(1)], [start2(2) end2(2)], 'r-');
            start3 = end2;
            end3 = start3 + [lengths(3)*cos(q(3)+q(2)+q(1)); lengths(3)*sin(q(3)+q(2)+q(1))];
            plot_qnew3 = plot([start3(1) end3(1)], [start3(2) end3(2)], 'r-');
            start4 = end3;
            end4 = start4 + [lengths(4)*cos(q(4)+q(3)+q(2)+q(1)); lengths(4)*sin(q(4)+q(3)+q(2)+q(1))];
            plot_qnew4 = plot([start4(1) end4(1)], [start4(2) end4(2)], 'r-');
            pause(0.01);
        elseif collision == 1
            % Delete plots of qnew
            delete(plot_qnew1); delete(plot_qnew2); delete(plot_qnew3); delete(plot_qnew4);
        end
    
        % Check if qnew is "near" qg
        dist_qnew_qg = sqrt((qnew(1)-qg(1))^2 + (qnew(2)-qg(2))^2 + (qnew(3)-qg(3))^2 + (qnew(4)-qg(4))^2);
        if dist_qnew_qg <= near_qg_thresh
            % Check collision along the edge from qnew to qg
            collision = 0;
            for t = 0:0.02:1
                q_inc = [qg(1) + t*(qnew(1)-qg(1));
                         qg(2) + t*(qnew(2)-qg(2));
                         qg(3) + t*(qnew(3)-qg(3));
                         qg(4) + t*(qnew(4)-qg(4))];
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
            % If qnew reaches qg: Add edge [qnew qg] to E, break out
            if collision == 0
                E{end+1} = [qnew, qg];
                near_qg = 1;    % this breaks out
            end
        end
    
    end
    
    pause(0.25);
    
    
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
                edge = E{i};
                if edge(1,2) == curr_q(1) && edge(2,2) == curr_q(2) && edge(3,2) == curr_q(3) && edge(4,2) == curr_q(4)
                    parent = edge(:,1);
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
    
        % Graph the path in blue:
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
        rainbow = colormap(hsv( width(path) + (width(path)-1)*25 ));
        rainbow_row = 1;
        for i = 1:length(path)-1
            q1 = path(:, i);
            q2 = path(:, i+1);
            
               for t = 0:0.04:1    % 50 increments between each 2 configurations
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

    
    else
        % IF FAILURE: reached max num_nodes. return path = 0
        path = 0;
    end
    
    
end

