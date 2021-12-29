function [model] = manipulator_object_model(G,LAMBDA_CONST)

import casadi.*

% system dimensions
nx = 21;               % last state models parameter
nu = 7;               % state noise on parameter
np = 225 + 63 + 6 + 1;%12;               % parameters

% dynamics
states = SX.sym('x', nx, 1);
controls = SX.sym('w', nu, 1);
p = SX.sym('p', np, 1); 

Mt = reshape(p(1:49),7,7);
Ct = reshape(p(50:56),7,1);
Nt = reshape(p(57:63),7,1);

x_dot_expr = [controls ; states(15:21) ; Mt\(- Ct - Nt + states(1:7))];
         
% constraint on lambda
J_ = reshape(p(64:105),6,7); %body jacobian

Mo = reshape(p(106:141),6,6);
Co = reshape(p(142:177),6,6);
No = reshape(p(178:183),6,1);

Jdot_ = reshape(p(184:225),6,7); %body jacobian dot

% dq_prev = reshape(p(226:232),7,1);

Mm = reshape(p(226:274),7,7);
Cm = reshape(p(275:281),7,1);
Nm = reshape(p(282:288),7,1);

%Fc = reshape(p(289:300),12,1); % contact force read from gazebo
Fb = reshape(p(289:294),6,1);

mu = p(295);
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

if (LAMBDA_CONST)
    % expr_h = pinv(G*Fc_hat)*(Mo*(J_*(states(15:21)-dq_prev)/dt) + ...
    %                 Co*J_*states(15:21) + No);
    % expr_h = pinv(G*Fc_hat)*(Mo*(J_*(states(15:21)-dq_prev)/dt + Jdot_*states(15:21)) + ...
    %                 No);
    
    ddq = Mm\(states(1:7) - J_'*Fb - Cm - Nm);
    ddx = J_*ddq + Jdot_*states(15:21);
    expr_h = pinv(G*Fc_hat)*(Mo*ddx + No);
end

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
if (LAMBDA_CONST)
    model.expr_h = expr_h;
end

end
