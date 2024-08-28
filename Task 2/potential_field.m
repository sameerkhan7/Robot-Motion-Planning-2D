function [q,n] = potential_field(qI, qG, Obs, A)
%function to run potential field for the stick

    %parameters
    steps = 2000;
    eta = 500;
    rho0 = 10;
    q = zeros(3,steps);
    q(:,1) = qI;
    alpha = 0.12;
    s = 0.06;
    dt = 0.1;
    
    %final configuration transformation
    thf = qG(3);
    Hg = [cos(thf) -sin(thf) qG(1); sin(thf) cos(thf) qG(2); 0 0 1];
    
    n = steps;

    for k=1:steps %2000 will be the maximum step
        grad_u = [0;0];
        total_torque = 0;
        for i=1:4
            th = q(3,k);
            H = [cos(th) -sin(th) q(1,k); sin(th) cos(th) q(2,k); 0 0 1];
            world_pt = H*[A(:,i);1]; %compute the points on the stick
            desired_pt = Hg*[A(:,i);1]; %desire points on gaol
            gradi = gradU_attr(world_pt(1:2),desired_pt(1:2),s) + gradU_rep(world_pt(1:2),Obs,rho0,eta); %compute gradient

            F = [-alpha*gradi;0]; %total force act on one point
            r = [A(:,i);0]; %length to center of mass
            torque = cross(r, F); %compute torque on one point
            total_torque = total_torque + torque;
            grad_u = grad_u + gradi;
        end
        
        q(1:2,k+1) = q(1:2,k) - alpha*grad_u; %x,y propagation
        tau = total_torque*dt; 
        q(3,k+1) = q(3,k) + tau(3); %add torque to current theta
        %make sure theta is with [-2pi, 2pi]
        if (q(3,k+1) >= 0)
            q(3,k+1) = mod(q(3,k+1),2*pi);
        elseif (q(3,k+1) < 0)
            q(3,k+1) = mod(q(3,k+1),-2*pi);
        end
        
        %breaking condition
        if norm(grad_u) < 1e-3 && norm(q(:,k+1)-qG) < 0.01
            n = k+1;
            break;
        end
    end

end

function U = U_attr(q,qf,s)
    U = 0.5*s*norm(q-qf)^2; %attractive force
end

function gradU = gradU_attr(q,qf,s)
    gradU = s*(q-qf); %attractive gradient
end

function U = U_rep(q,O,rho0,eta)
    U = 0;
    for i=1:size(O,2)
        C = O{i};
        d = getDistance(q,C);
        if (d < 0 || d > rho0)
            continue
        end
        U = U + (eta/2)*(1/d-1/rho0)^2; %replusion force
    end
end

function gradU = gradU_rep(q,O,rho0, eta)
     gradU = [0; 0];
     for i=1:size(O,2)
        C_obs = O{i};
        [d,point] = getDistance(q,C_obs);
%         if (d < 0)
%              disp(i)
%         end
        if (d < 0 || d > rho0)
            continue
        end
        gradrho = (q-point)/norm(q-point);
        gradU_i = (eta/d^2)*(1/rho0-1/d)*gradrho;
        gradU = gradU + gradU_i;
     end
end

function [dist,point] = getDistance(q,C)
     xv = C(1,:); yv = C(2,:);
     is_in = inpolygon(q(1,1),q(2,1),xv,yv);
     if (is_in)
         %disp('in')
         dist = -1;
         point = [-1;-1];
         return;
     else
        mind = Inf;
        for k=1:size(C,2) %loop through each side of the polygon
            if (k==size(C,2))
                vec = C(:,1)-C(:,k);
                 i1 = k; i2 = 1;
            else
                vec = C(:,k+1)-C(:,k);
                 i1 = k; i2 = k+1;
            end
            d = norm(vec);
            a = q-C(:,k);
            length = dot(a,vec)/d; %get the projection distance
            if (length <= 0)
                cpt = C(:,i1);
            elseif (length >= d)
                cpt = C(:,i2);
            else
                cpt = C(:,i1) + vec.*(length/d);
            end
            if (norm(cpt-q) < mind)
                mind = norm(cpt-q);
                point = cpt;
            end
        end

        dist = mind;
     end
   
end