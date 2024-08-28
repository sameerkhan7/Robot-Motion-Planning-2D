function [q_new,u] = findNextState(q_near,q_rand,u_phi,u_set,dt,r,x_max,y_max)
% finds the input u in input set u_set that makes closest movement from
% q_near to q_rand in time step dt

%test all inputs in set
dist = inf;
for i=1:length(u_set)
    %calculate q_new using input u_k
    u_k = u_set(i);
    q_dt = [r*u_phi*cos(q_near(3));r*u_phi*sin(q_near(3));u_k]*dt;
    q_k = q_near + q_dt;
    %bound the angle to -pi to pi
    q_k(3) = wrapToPi(q_k(3));
    %check if u_k creates closest q_new to q_rand
    dist_k = norm(q_k-q_rand);
    if dist_k < dist
        q_new = q_k;
        u = u_k;
        dist = dist_k;
    end
end

end

