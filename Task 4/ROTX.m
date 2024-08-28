% Mariana Smith (msmit458)
% Lab 1
% Task 2.1
% Write a function ROTX() which accepts a scalar roll value (in 
% radians) and returns the corresponding 3x3 rotation matrix.

function [Rx] = ROTX(roll)

Rx = [1, 0, 0;...
    0, cos(roll), -sin(roll);...
    0, sin(roll), cos(roll)];


end