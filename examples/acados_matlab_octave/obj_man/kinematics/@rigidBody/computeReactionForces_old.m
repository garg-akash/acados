function Fc = computeReactionForces(obj, Fb)
%COMPUTE_REACTION_FORCES Summary of this function goes here
%   Detailed explanation goes here

lb = [-Inf -Inf 0.0];
ub = [Inf Inf Inf];

Lb = [lb lb lb lb zeros(1,16)];
Ub = [ub ub ub ub, inf(1,16, 'double')];

a = obj.x_hat;
Aeq = blkdiag(obj.G, a, a, a, a); % 
Aeq(7:end,1:12) = -eye(12);
Beq = [-Fb; zeros(12,1)];

A = zeros(28);
A(13:end,13:end) = -eye(16);
B = zeros(28,1);

% min_Fc Fc'*H*Fc
% s.t.  A*Fc < b // Fcx < a*xhat1
%       G*Fc = Fb        
% [Fc,fval,exitflag,output,lambda] = quadprog(eye(12,12), zeros(12,1), ...
%    A, B, obj.G, -Fb);%, Lb, Ub);

% min_Fc Fc'*H*Fc
% s.t. A*[Fc, lambda]' = [Fb, zero]'        

% [x,fval,exitflag,output,lambda] = quadprog(eye(28,28), zeros(28,1), ...
%     A, B, Aeq, Beq);%, Lb, Ub);
[x,fval,exitflag,output,lambda] = quadprog(eye(28,28), zeros(28,1), ...
    [], [], Aeq, Beq, Lb, Ub);

if(exitflag > 0)
    obj.Fc = x(1:12,1);
else
    Fc = -pinv(obj.G)*Fb;
    hr = line(10*[Fc(2) 0],10*[Fc(3) 0],'color',[1 0 0]);
    for i=0:3
        t = Fc(i*3+2,1);
        if (t < 0)
            ps = dot(Fc(i*3+1:(i+1)*3,1),x_hat_1);
            obj.Fc(i*3+1:(i+1)*3,1) = x_hat_1*ps;
        else
            ps = dot(Fc(i*3+1:(i+1)*3,1), x_hat_2);
            obj.Fc(i*3+1:(i+1)*3,1) = x_hat_2*ps;
        end
    end
end

Fc = obj.Fc;

h_fig = figure(1)
h = plotRB(obj, h_fig, 1, Fb, Fc);
drawnow
delete(h)

end

