function [model] = rodyman_object_model_lambda(G)

import casadi.*

% system dimensions
nx = 27 + 16;           % state models parameter (tau, qdot, q, lambda)
nu = 9;                 % state noise on parameter
np = 391;               % parameters

% dynamics
states = SX.sym('x', nx, 1);
controls = SX.sym('w', nu, 1);
p = SX.sym('p', np, 1); 

Mt = reshape(p(1:81),9,9);
Ct = reshape(p(82:90),9,1);
Nt = reshape(p(91:99),9,1);

J_ = reshape(p(100:153),6,9); %body jacobian
Mo = reshape(p(154:189),6,6);
Co = reshape(p(190:225),6,6);
No = reshape(p(226:231),6,1);
Jdot_ = reshape(p(232:285),6,9);  

Mm = reshape(p(286:366),9,9);
Cm = reshape(p(367:375),9,1);
Nm = reshape(p(376:384),9,1);
Fb = reshape(p(385:390),6,1);

mu = p(391);
theta_ = atan(mu);
z_hat_ = [0,0,1]';
Rx = [1 0 0; 0 cos(theta_) -sin(theta_); 0 sin(theta_) cos(theta_)];
Rx_ = [1 0 0; 0 cos(-theta_) -sin(-theta_); 0 sin(-theta_) cos(-theta_)];
Ry = [cos(theta_) 0 sin(theta_); 0 1 0; -sin(theta_) 0 cos(theta_)];
Ry_ = [cos(-theta_) 0 sin(-theta_); 0 1 0; -sin(-theta_) 0 cos(-theta_)];
x_hat_ = [Rx*z_hat_, ...
        Rx_*z_hat_, ...
        Ry*z_hat_, ...
        Ry_*z_hat_];
Fc_hat = blkdiag(x_hat_, x_hat_, x_hat_, x_hat_);

x_dot_expr = [controls ; ...
              states(19:27) ; ... 
              Mt\(- Ct - Nt + states(1:9)); ...
              pinv(G*Fc_hat)*(Mo*(Jdot_*pinv(Mt)*(states(1:9) - Ct - Nt) + ...
              J_*pinv(Mt)*controls) + Jdot_*states(19:27))];

% store eveything in model struct
model = struct();
model.nx = nx;
model.nu = nu;
model.np = np;
% model.ny = ny;

model.sym_x = states;
model.sym_u = controls;
model.sym_p = p;
model.expr_f_expl = x_dot_expr;

end
