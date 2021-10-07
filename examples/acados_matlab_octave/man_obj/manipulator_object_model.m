function [model] = manipulator_object_model(G,Fc_hat,dt,LAMBDA_CONST)

import casadi.*

% system dimensions
nx = 21;               % last state models parameter
nu = 7;               % state noise on parameter
np = 190 + 42;               % parameters

% dynamics
states = SX.sym('x', nx, 1);
controls = SX.sym('w', nu, 1);
p = SX.sym('p', np, 1); 

x_dot_expr = [controls ; states(15:21) ; reshape(p(1:49),7,7)\(-reshape(p(50:56),7,1) ...
             -reshape(p(57:63),7,1)+states(1:7))];

% constraint on lambda
J_ = reshape(p(64:105),6,7); %body jacobian
Mo = reshape(p(106:141),6,6);
Co = reshape(p(142:177),6,6);
No = reshape(p(178:183),6,1);
dq_prev = reshape(p(184:190),7,1);  
Jdot_ = reshape(p(191:232),6,7); %body jacobian dot
if (LAMBDA_CONST)
%     expr_h = pinv(G*Fc_hat)*(Mo*(J_*(states(15:21)-dq_prev)/dt) + ...
%                 Co*J_*states(15:21) + No);
expr_h = pinv(G*Fc_hat)*(Mo*(J_*(states(15:21)-dq_prev)/dt + Jdot_*states(15:21)) + ...
                No);
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