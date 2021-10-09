function [model] = object_manipulator_model(G, Fc_hat, dt, TAU_CONST)

import casadi.*

% system dimensions
nx = 29;               % last state models parameter
nu = 16;               % state noise on parameter
np = 78 + 42 + 48 + 6;               % parameters

% dynamics
states = SX.sym('x', nx, 1);
controls = SX.sym('w', nu, 1);
p = SX.sym('p', np, 1); 

x_dot_expr = [zyx2R(states(4:6))*states(7:9); zyx2E(states(4:6))*states(10:12);...
              reshape(p(1:36),6,6)\(-reshape(p(37:72),6,6)*states(7:12)-reshape(p(73:78),6,1)+ ...
              G*Fc_hat*states(14:29)); 0; controls];

Jb_t = reshape(p(79:120),7,6); %body jacobian transpose
M_t = reshape(p(121:156),6,6);
C_t = reshape(p(157:162),6,1);
N_t = reshape(p(163:168),6,1);
v_prev = reshape(p(169:174),6,1);  
if (TAU_CONST)
    expr_h = Jb_t*(M_t*(states(7:12) - v_prev)/dt + C_t + N_t);
end

  
% store eveything in model struct
model = struct();
model.nx = nx;
model.nu = nu;
model.np = np;

model.sym_x = states;
model.sym_u = controls;
model.sym_p = p;
model.expr_f_expl = x_dot_expr;
if (TAU_CONST)
    model.expr_h = expr_h;
end

end
