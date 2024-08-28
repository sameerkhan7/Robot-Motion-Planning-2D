clc;clear;close all;

%bounds of problem
bounds = [0  100 100 0; 0 0 100 100];

%obstacles
CB1 =[30 60 60 30; 20 20 50 50];
CB2 =[50 80 40; 75 100 100];
CB3 =[80 100 100 80;60 60 70 70];
B = {CB1, CB2, CB3};

qI =[5; 50; 0]; %initial state of center of robot [x y theta]
qG =[90; 40; 0]; %goal state of center of robot
R = [0 4 0; -1 0 1]; %triangular robot points w.r.t origin of robot

%model parameters
alpha = .3;
dt=.5;
n = 2000;
r = 2;

%run RRT model
[path,V,E] = flexibleNeedleModel(alpha,qI,qG,n,dt,r,R(:,2),B,max(bounds(1,:)),max(bounds(2,:)));

%plots
figure(1)
axis equal
hold on
plot(qI(1),qI(2),'bo')
plot(qG(1),qG(2),'rx')
P = [];
for i=1:length(B)
    C = B{i};
    P = [P,polyshape(C(1,:),C(2,:))];
    plot(P(i))
end
%plot boundary
bounds = [bounds,bounds(:,1)];
for i=1:length(bounds)-1
    plot(bounds(:,i),bounds(:,i+1),"k")
end

%plot robot planning
scatter(V(1,:),V(2,:),'.k')
if length(path(1,:)) ~= 1 %path exists
    for i=1:length(path(1,:))
        q = path(:,i);
        rot = rot2d(q(3));
        R_i = rot*R+q(1:2);
        plot(polyshape(R_i(1,:),R_i(2,:)),"FaceColor","green")
        pause(.02)
    end
end



