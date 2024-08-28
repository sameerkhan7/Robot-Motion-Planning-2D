% Mariana Smith (msmit458)
% Lab 1
% Task 2.2
% Write a function ROTY() which accepts a scalar roll value (in 
% radians) and returns the corresponding 3x3 rotation matrix.

function [Ry] = ROTY(pitch)

Ry = [cos(pitch), 0, sin(pitch);...
    0, 1, 0;...
    -sin(pitch), 0, cos(pitch)];
    
end