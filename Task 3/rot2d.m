function [rot] = rot2d(theta)
%xy rotation matrix by an angle theta (radians)
rot = [cos(theta) -sin(theta); sin(theta) cos(theta)];
end

