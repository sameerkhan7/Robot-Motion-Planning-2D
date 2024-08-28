clear all;
close all;

lengths = [89; 130; 425; 120; 392; 110; 95; 83];
xmax = 1010;
ymax = 1010;
zmax = 1010;
yobs = 200;
xobs = 200;
q_home = [-pi; 0; 0; 0; 0; 0; 0];
qi = q_home + [4*pi/5;   pi/4;     -pi/3;  pi/4;   -pi/4;   0;   -pi/4];
qg = q_home + [0.7*pi;   pi/2;     -pi/8;  pi/4;  pi/4;   0;   -pi/4];
dq = 0.8;
near_qg_thresh = 0.5;

[V, E, path] = project_4(lengths, xmax, ymax, zmax, yobs, xobs, qi, qg, dq, near_qg_thresh);

