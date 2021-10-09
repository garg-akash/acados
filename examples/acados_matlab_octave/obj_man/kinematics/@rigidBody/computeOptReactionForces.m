function [Fb_, Fc_, lambda_, flag_] = computeOptReactionForces(obj, Fb)
%COMPUTE_REACTION_FORCES Summary of this function goes here
%   Detailed explanation goes here

% x = [Fb_tilde, Fc, lambda]'

% Equality constraints 
a = obj.contacts(1).x_hat_;
Aeq = zeros(22,34);
Aeq(1:6, 1:6) = eye(6);
Aeq(1:6, 7:18) = obj.G; 
Aeq(7:18, 7:18) = -eye(12);
Aeq(7:18, 19:end)= blkdiag(a, a, a, a);
Aeq(19, 3) = 0;
Aeq(20, 4) = 1;
Aeq(21, 5) = 1;
Aeq(22, 6) = 1;
Beq = [Fb; zeros(12,1); 0; 0; 0; 0];

% Inequality constraints
A = [zeros(16, 18), -eye(16)];
B = zeros(16,1);

% Cost function
cost = 0.1*eye(34, 34);
cost(1:3,1:3) = 2000*eye(3);
cost(4:6,4:6) = 0*eye(3);
cost(19:end, 19:end) = zeros(16, 16);

% min_(Fb_tilde, Fc, lambda) norm(Fb_tilde) + norm(Fc) + norm(lambda)
% s.t. G*Fc* = Fb*
%      Fc* = lambda*xhat
%      lambda* >= 0    

% [x,fval,exitflag,output,lambda] = quadprog(eye(34,34), zeros(34,1), ...
%     A, B, Aeq, Beq);%, Lb, Ub);
[x,fval,exitflag,output,lambda] = quadprog(cost, [], A, B, Aeq, Beq, [], []);

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

% Fc = [];
% for i = 1: size(obj.contacts, 2)
%     Fc = [Fc; obj.contacts(i).Fc_];
% end
% 
% alpha_p = obj.getAlphaVector();
% alpha = obj.computeAlphaVector(Fc_);
% alpha_dot = (alpha-alpha_p)/0.005; %%%%%%
% h = 0;
% dh_da = [];
% dh_dadot = [];
% for i = 1 : size(alpha,1)
%     h = h + H(alpha(i), obj.contacts(1).theta_, 5) + H(alpha_dot(i), 10, 5);
%     dh_dadot = [dh_dadot; dH_da(alpha_dot(i), 10, 5)];
%     dh_da = [dh_da; dH_da(alpha(i), obj.contacts(1).theta_, 5)];
% end
% 
% h = h - 19.2;
% 
% % x = [alphadd, taub, Fco, lambda]'
% % Equality constraints 
% a = obj.contacts(1).x_hat_;
% Aeq = zeros(21,43);
% Aeq(1:3, 1:12) = -obj.I*obj.Go;
% Aeq(1:3, 13:15) = -eye(3);
% Aeq(4:6, 13:15) = eye(3); 
% Aeq(4:6, 16:27) = -obj.G(4:6,:);             
% Aeq(7:18, 16:27) = -eye(12);
% Aeq(7:18, 28:43) = blkdiag(a, a, a, a);
% Aeq(19:21, 16:27) = obj.G(1:3,:);
% Beq = zeros(21,1);
% Beq(19:21) = Fb_(1:3);
% 
% % Inequality constraints
% A = zeros(17,43);
% A(1, 1:12) = dh_dadot';    % alphadotdot
% A(2:17, 28:43) = -eye(16); % Lambda
% B = zeros(17,1);
% B(1,1) = -dh_da'*alpha_dot-0.01*h;
% 
% % Cost function
% cost = zeros(43, 43);
% cost(1:12, 1:12) = 0.01*eye(12);
% cost(16:27, 16:27) = 0.2*eye(12, 12);
% 
% % min_(Fb_tilde, Fc, lambda) norm(Fb_tilde) + norm(Fc) + norm(lambda)
% % s.t. G*Fc* = Fb*
% %      Fc* = lambda*xhat
% %      lambda* >= 0    
% 
% % [x,fval,exitflag,output,lambda] = quadprog(eye(34,34), zeros(34,1), ...
% %     A, B, Aeq, Beq);%, Lb, Ub);
% [x, fval, exitflag, output, lambda] = quadprog(cost, [], A, B, Aeq, Beq, [], []);
% 
% if(exitflag > 0)
%     flag_ = 0;
%     taub_ = x(13:15);
%     Fco_ = x(16:27,1);
%     lambda_ = x(28:end);
% else
%     flag_ = 1;
%     taub_ = zeros(3,1);
%     Fco_ = zeros(12,1);
%     lambda_ = zeros(28,1);
% end
% 
% Fc_ = Fc_+Fco_;

obj.setContactForces(Fc_);

end

