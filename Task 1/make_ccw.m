% Reorder a matrix of vertices in CCW order
% Mariana Smith, RMP Homework 7

function M_ccw = make_ccw(M)

    matrix = M;
    x = matrix(1,:);
    y = matrix(2,:);
    angles = atan2(y - mean(y), x - mean(x));
    [~, idx] = sort(angles);
    M_ccw = matrix(:, idx);

end