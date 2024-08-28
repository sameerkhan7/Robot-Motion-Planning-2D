function v = nearest_vertex_serial(q,V)
    min_dist = inf;
    v = [];
    for i = 1:width(V)
        vi = V{i};
        dist = sqrt((q(1)-vi(1))^2 + (q(2)-vi(2))^2 + (q(3)-vi(3))^2 + (q(4)-vi(4))^2);
        if dist < min_dist
            v = vi;
            min_dist = dist;
        end
    end
end