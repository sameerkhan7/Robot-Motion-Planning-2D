% Mariana Smith (msmit458)
% Lab 1
% Task 2.2
% Write a function ROTZ() which accepts a scalar roll value (in 
% radians) and returns the corresponding 3x3 rotation matrix.

function [Rz] = ROTZ(yaw)

Rz = [cos(yaw), -sin(yaw), 0;...
    sin(yaw), cos(yaw), 0;...
    0, 0, 1];
    
end