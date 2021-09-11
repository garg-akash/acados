function [model] = manipulator_model()

import casadi.*

% system dimensions
nx = 21;               % last state models parameter
nu = 7;               % state noise on parameter
np = 63;               % parameters

% dynamics
states = SX.sym('x', nx, 1);
controls = SX.sym('w', nu, 1);
p = SX.sym('p', np, 1); 

x_dot_expr = [controls ; states(15:21) ; reshape(p(1:49),7,7)*(-reshape(p(50:56),7,1) ...
             -reshape(p(57:63),7,1)+states(1:7))];
  
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
