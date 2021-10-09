function [Fb_, Fc_, lambda_, flag_] = computeReactionForces(obj, Fb)
%COMPUTE_REACTION_FORCES Summary of this function goes here
%   Detailed explanation goes here

% x = [Fb_tilde, Fc, lambda]'

% Bound constraints
lb = [-inf(1,3, 'double')];
ub = inf(1,3, 'double');
Lb = [-inf(1,6, 'double'), lb lb lb lb, zeros(1,16)]; % 1x34
Ub = [inf(1,6, 'double'),  ub ub ub ub, inf(1,16, 'double')]; % 1x34

% Equality constraints 
a = obj.contacts(1).x_hat_;
Aeq = zeros(22,34);
Aeq(1:6, 1:6) = eye(6);
Aeq(1:6, 7:18) = obj.G; 
Aeq(7:18, 7:18) = -eye(12);
Aeq(7:18, 19:end)= blkdiag(a, a, a, a);
Aeq(19, 3) = 1;
Aeq(20, 4) = 1;
Aeq(21, 5) = 1;
Aeq(22, 6) = 1;
Beq = [Fb; zeros(12,1); 0; 0; 0; 0];

% Inequality constraints
A = [zeros(16, 18), -eye(16)];
B = zeros(16,1);

% Cost function
H = 0.1*eye(34, 34);
H(1:3,1:3) = 200*eye(3);
H(4:6,4:6) = 0*eye(3);
H(19:end, 19:end) = zeros(16, 16);

% min_(Fb_tilde, Fc, lambda) norm(Fb_tilde) + norm(Fc) + norm(lambda)
% s.t. G*Fc* = Fb*
%      Fc* = lambda*xhat
%      lambda* >= 0    

% [x,fval,exitflag,output,lambda] = quadprog(eye(34,34), zeros(34,1), ...
%     A, B, Aeq, Beq);%, Lb, Ub);
[x,fval,exitflag,output,lambda] = quadprog(H, [], A, B, Aeq, Beq, [], []);

if(exitflag > 0)
    flag_ = 0;
    Fb_ = x(1:6);
    Fc_ = x(7:18,1);
    lambda_ = x(19:end);
else
    flag_ = 1;
    Fc_ = zeros(12,1);
    Fb_ = zeros(6:1);
    lambda_ = zeros(16:1);
end

obj.setContactForces(Fc_);

end

