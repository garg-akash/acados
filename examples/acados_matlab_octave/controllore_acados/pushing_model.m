function model = pushing_model()

import casadi.*

%% system dimensions
nx = 4;
nu = 4;

%% system parameters
L=[0.1156,0,0;0,0.1154,0;0,0,0.2129]; %%limit surface ellipsoid
mu=0.2;  %%friction coefficient

%% named symbolic variables
xp = SX.sym('xp');                     % x position
yp = SX.sym('yp');                     % y position
orientation= SX.sym('orientation');% slider orientation
pusherangle = SX.sym('pusherangle');        % pusher angle
fn = SX.sym('fn');                     % normal force
ft = SX.sym('ft');                     % tangential force
phiplus = SX.sym('phiplus');              % phi dot plus
phiminus = SX.sym('phiminus');             % phi dot minus

%% (unnamed) symbolic variables
sym_x = vertcat(xp, yp, orientation, pusherangle);
sym_xdot = SX.sym('xdot', nx, 1);
sym_u = vertcat(fn, ft, phiplus, phiminus);
sym_p = SX.sym('p', 13, 1);

%% dynamics
Xc=-0.5;
m=tan(pusherangle);
Yc=-Xc*m;
R=[cos(orientation),-sin(orientation),0;sin(orientation),cos(orientation),0;0,0,1];
J=[1,0,-Yc;0,1,Xc];
femp=(R)*L*(J');
funcmat=[femp(1,1),femp(1,2),0,0;femp(2,1),femp(2,2),0,0;femp(3,1),femp(3,2),0,0;0,0,1,-1];
expr_f_expl=funcmat*sym_u;
expr_f_impl = expr_f_expl - sym_xdot;

lambda=[mu*fn-ft;mu*fn+ft];
z=lambda'*[phiplus;phiminus]; %%slack variable


%% constraints
expr_u = sym_u;
expr_x = sym_x;
expr_constraint_1=(mu*fn-ft);
expr_constraint_2=(mu*fn+ft);

expr_h = vertcat(expr_x, expr_u, expr_constraint_1,expr_constraint_2);
%% cost
Wx=[sym_p(6),0,0,0;0,sym_p(7),0,0;0,0,sym_p(8),0;0,0,0,sym_p(13)];
Wxn=sym_p(12)*Wx;
Wu=(0.01)*[sym_p(9),0,0,0;0,sym_p(10),0,0;0,0,sym_p(11),0;0,0,0,sym_p(11)];
Wz=sym_p(5);
xstar=[sym_p(1); sym_p(2);sym_p(3); sym_p(4)];
expr_ext_cost_f = (sym_x-xstar)'* Wxn * (sym_x-xstar);
expr_ext_cost = ((sym_x-xstar)'* Wx * (sym_x-xstar))  + (sym_u' * Wu * sym_u) +  Wz*(z^2);

% % nonlinear least sqares
% cost_expr_y = vertcat(sym_x, sym_u);
% W = blkdiag(W_x, W_u);
% model.cost_expr_y_e = sym_x;
% model.W_e = W_x;



%% populate structure
model.nx = nx;
model.nu = nu;
model.sym_x = sym_x;
model.sym_p = sym_p;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.sym_z = z;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
model.expr_h = expr_h;
model.expr_ext_cost = expr_ext_cost;
model.expr_ext_cost_f = expr_ext_cost_f;
% 
% model.cost_expr_y = cost_expr_y;
% model.W = W;

end