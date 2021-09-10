function [model] = object_model(dyn, G, Fc_hat)

import casadi.*

% system dimensions
nx = 29;               % last state models parameter
nu = 16;               % state noise on parameter
% ny = 1;

% weighting matrices
% Q = blkdiag(10,10,10,10,10,10,1,1,1,1e-2,1e-2,1e-2,0,1e-4*eye(16));
% R = 1e-4*eye(nu);
% P0 = 0.1*eye(nx);
% P0(nx, nx) = 0.001;

% dynamics
states = SX.sym('x', nx, 1);
controls = SX.sym('w', nu, 1);

x_dot_expr = [zyx2R(states(4:6))*states(7:9); zyx2E(states(4:6))*states(10:12);...
              reshape(dyn(1:36),6,6)*(-reshape(dyn(37:72),6,6)*states(7:12)-reshape(dyn(73:end),6,1)+ ...
              G*Fc_hat*states(14:29)); 0; controls];

  
% store eveything in model struct
model = struct();
model.nx = nx;
model.nu = nu;
% model.ny = ny;

model.sym_x = states;
model.sym_u = controls;
model.expr_f_expl = x_dot_expr;

% model.W_0 = blkdiag(R, Q, P0);
% model.W = blkdiag(Q, R);

% model.h = 0.01;
% model.N = 5;     % MHE horizon

end
