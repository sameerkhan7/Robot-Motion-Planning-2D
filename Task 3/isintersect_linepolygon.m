function [intersects] = isintersect_linepolygon(p0,p1,Q)
% find if a point/line S intersects polygon Q from PDF
%line segment p0 -> p1, Q = 2xN cell array of N-vertex polygon

num = length(Q(1,:));
Q = [Q,Q(:,1)];
%if S is just a point
if isequal(p0,p1)
    intersects = inpolygon(p0(1),p0(2),Q(1,:),Q(2,:));
    return
else
    tE = 0;
    tL = 1;
    ds = p1 - p0;
    for i=1:num %loop through all edges
        q_i = Q(:,i);
        q_i1 = Q(:,i+1);
        e = q_i1 - q_i;
        %outward normal if ccw
        n = [e(2),-e(1)]/(norm(e));
        N = dot(-(p0 - q_i),n);
        D = dot(ds, n);

        if D == 0
            if N < 0
                intersects = false;
                return
            end
        end
        
        t = N/D;
        if D < 0
            tE = max(tE,t);
            if tE > tL
                intersects = false;
                return
            end
        elseif D > 0
            tL = min(tL,t);
            if tL < tE
                intersects = false;
                return
            end
        end
    end
    if tE <= tL
        intersects = true;
        return
    else
        intersects = false;
        return
    end
end
%intersects = "failed";
end

