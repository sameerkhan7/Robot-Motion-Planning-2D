%Task 2 main script for running

clear all; close all;

%world

B = cell(1,3);
B1 = [0 0 20; 15 35 20];
B2 = [30 50 50; 15 25 5];
B3 = [30 50 50 30; 30 30 45 45];

B{1} = B1;
B{2} = B2;
B{3} = B3;

stick_initial = [5 5 15 15; 7 5 5 7];
stick_goal = [10 10 20 20; 44 42 42 44];

figure;
plot(polyshape(B1(1,:),B1(2,:)));
hold on;
plot(polyshape(B2(1,:),B2(2,:)));
plot(polyshape(B3(1,:),B3(2,:)));

patch(stick_initial(1,:),stick_initial(2,:),'r-');
patch(stick_goal(1,:),stick_goal(2,:),'b-');
axis([0 50 0 50]);

qI = [10;6;0]; qG = [15;43;0];

cltr_points = [stick_initial(:,1)-qI(1:2), stick_initial(:,2)-qI(1:2),stick_initial(:,3)-qI(1:2), stick_initial(:,4)-qI(1:2)];

A = cltr_points;
[q,n] = potential_field(qI, qG, B, A);
fprintf('APF Planning took %d steps\n', n);

%plot all the sticks
disp('start drawing robots')
for k=1:n
    th = q(3,k);
    H = [cos(th) -sin(th) q(1,k); sin(th) cos(th) q(2,k); 0 0 1];
    p = [cltr_points;ones(1,4)];
    world_pts = H*p;
    x = world_pts(1,:); y = world_pts(2,:);
    plot(polyshape(x,y),'EdgeColor','b','FaceColor','w');
    M(k) = getframe;
end
disp('finished drawing robots')

% v = VideoWriter('task2.avi');
% v.FrameRate = 30; %slow down the playback of the video
% open(v)
% writeVideo(v, M)
% close(v)

