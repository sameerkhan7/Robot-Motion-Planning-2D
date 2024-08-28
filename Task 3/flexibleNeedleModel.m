function [path,V,E] = flexibleNeedleModel(alpha,qI,qG,n,dt,r,tip,B,x_max,y_max)
%use the non-holonomic unicycle model and RRT to path plan for a flexible needle
%Inputs:
%   alpha: robot rotation rate parameter
%   qI: initial state
%   qG: goal state
%   n:  max number of iterations for RRT
%   dt: time step
%   r: wheel radius
%   tip: body frame location of tip of needle
%   B: obstacle cell array
%   x_max: max x bound 
%   y_max: max y bound
%Outputs:
%   path: path of states from qI to Xg
%   V: all vertices of RRT
%   E: edges of RRT in parent/child form with inputs to each state

u_phi = 1; % wheel rotation rate input
u_w = [0, alpha, -alpha]; % robot rotation rate inputs

% RRT algorithm for a planar point robot in R2
num_obs = length(B);
V = qI; % 2xN points
E = [1;NaN]; %2xN of parent indices and inputs for each edge
for i=1:n %loop n times
    %find random point in C
    q_rand = [rand(1)*x_max;rand(1)*y_max;rand(1)*2*pi-pi];
    %find closest q in V to q_rand by distance (no angle)
    [q_near,i_near] = nearest_vertex(q_rand,V);
    %create new q and find best input u from set
    [q_new,u] = findNextState(q_near,q_rand,u_phi,u_w,dt,r,x_max,y_max);
    %find tip of needle at q_new
    rot = rot2d(q_new(3));
    q_tip = rot*tip + q_new(1:2);
    %check if q_tip is inside of bounds
    in_bounds = true;
    if(q_tip(1)>x_max || q_tip(1)<0 || q_tip(2)>y_max || q_tip(2)<0)
        in_bounds = false;
    end
    %check if q_new needle tip and its edge are in collision
    collision = false;
    for k=1:num_obs
        %q_new is in collision
        if isintersect_linepolygon(q_near(1:2),q_tip,B{k})
            collision = true;
        end
    end
    %add q_new to G
    if ~collision && in_bounds
        V = [V,q_new];
        E = [E,[i_near;u]];
    end
    % goal set is a radius or Xg around qG
    Xg = 3;
    if norm(qG(1:2)-q_tip) < Xg
        %find path from qI to qG set
        path = find_path_RRT(1,length(V(1,:)),V,E(1,:));
        return
    end
end
disp("Did not find path")
path = 0;
end

