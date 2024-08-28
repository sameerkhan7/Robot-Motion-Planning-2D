clear all;
close all;
n = 4;
lengths = [5; 5; 5; 5];
xmax = 20;
ymax = 20;
B = {[-8 -2 -5; 10 10 15], [-12 -6 -6 -12; -12 -12 -7 -7], [8 12 12 8; 4 4 7 7], [13 17 18 12; -11 -11 -8 -8]};
qi = [3*pi/4; pi/5; -pi/3; -pi/5];
qg = [-2*pi/5; pi/4; -pi/5; pi/5];
num_vertices = 150;
K = 6;
[V_prm, E_prm, path_prm] = project_1a_prm(lengths, xmax, ymax, B, qi, qg, num_vertices, K);

clear all;
n = 4;
lengths = [5; 5; 5; 5];
xmax = 20;
ymax = 20;
B = {[-8 -2 -5; 10 10 15], [-12 -6 -6 -12; -12 -12 -7 -7], [8 12 12 8; 4 4 7 7], [13 17 18 12; -11 -11 -8 -8]};
qi = [3*pi/4; pi/5; -pi/3; -pi/5];
qg = [-2*pi/5; pi/4; -pi/5; pi/5];
dq = 0.5;
near_qg_thresh = 2;
[V_rrt, E_rrt, path_rrt] = project_1b_rrt(lengths, xmax, ymax, B, qi, qg, dq, near_qg_thresh);




