function tau_ = computeOptTau(obj, alpha, alpha_dot)
%computeOptTau Summary of this function goes here
%   Detailed explanation goes here

h = 0;
dh_da = [];
dh_dadot = [];
for i = 1 : size(alpha,1)
    h = h + H(alpha(i), obj.contacts(1).theta_, 5) + H(alpha_dot(i), 1000, 5);
    dh_dadot = [dh_dadot; dH_da(alpha_dot(i), 1000, 5)];
    dh_da = [dh_da; dH_da(alpha(i), obj.contacts(1).theta_+0.1, 5)];
end
h = h - 19.2;

% x = [alphadd, taub]'
% Equality constraints
Aeq = zeros(3, 15);
Aeq(1:3, 1:12) = -obj.I*obj.Go;
Aeq(1:3, 13:15) = -eye(3);
Beq = zeros(3,1);

% Inequality constraints
A = zeros(1,15);
A(1, 1:12) = dh_dadot';    % alphadotdot
B = -dh_da'*alpha_dot-10*h;

% Cost function
cost = zeros(15, 15);
cost(1:12, 1:12) = 100*eye(12);
cost(13:15, 13:15) = 0.1*eye(3);

% min_(Fb_tilde, Fc, lambda) norm(Fb_tilde) + norm(Fc) + norm(lambda)
% s.t. G*Fc* = Fb*
%      Fc* = lambda*xhat
%      lambda* >= 0

% [x,fval,exitflag,output,lambda] = quadprog(eye(34,34), zeros(34,1), ...
%     A, B, Aeq, Beq);%, Lb, Ub);
[x, fval, exitflag, output, lambda] = quadprog(cost, [], A, B, Aeq, Beq, [], []);

if(exitflag > 0)
    flag_ = 0;
    tau_ = x(13:15);
else
    flag_ = 1;
    tau_ = zeros(3,1);
end

end

