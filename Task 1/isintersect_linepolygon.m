function b = isintersect_linepolygon(S,Q)
    % S: 2D segment from p0 to p1 (I'll assume this is a 2x2 matrix [p0 p1]
    % Q: convex poly 2xn vertices
    % b=1 true if intersection; b=0 false if free
    
    Q = make_ccw(Q);
    Q(:,end+1) = Q(:,1);

    p0 = S(:,1);
    p1 = S(:,2);

    if isequal(p0, p1) % If S is single point
        b = inpolygon(p0(1), p0(2), Q(1,:), Q(2,:));
    else
        te = 0;
        tl = 1;
        ds = p1 - p0;
        b = zeros(1, size(Q,2) - 1);
        for i = 1:length(Q)-1
            % Compute outward normal n of each edge e
            q1 = Q(:,i);
            q2 = Q(:,i+1);
            dx = q2(1) - q1(1);
            dy = q2(2) - q1(2);
            n = [dy; -dx]/norm([dy; -dx]);

            N = dot(-(p0 - q1), n);
            D = dot(ds, n);

            % Check for parallel lines
            if D == 0
                if N < 0
                    % S on opposite side of edge
                    b(i) = 0;
                end
            else
                t = N/D;
                if D < 0
                    % Segment enters from outside
                    te = max(te, t);
                elseif D > 0
                    % Segment exits from inside
                    tl = min(tl, t);
                end
            end

        end

        if te <= tl
            b(end+1) = 1;
        else
            b(end+1) = 0;
        end

    end % if

    if ismember(1, b)
        b = 1;
    else
        b = 0;
    end


    
end









