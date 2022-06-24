clear all
check_acados_requirements()

%% discretization
N = 25;
T = 10; % time horizon length
x0 = [0;0;0;pi];
Xstar = [1;-1;pi/4;pi];

nlp_solver = 'sqp'; % sqp, sqp_rti
qp_solver = 'partial_condensing_hpipm';
    % full_condensing_hpipm, partial_condensing_hpipm, full_condensing_qpoases
qp_solver_cond_N = 5; % for partial condensing
% integrator type
sim_method = 'erk'; % erk, irk, irk_gnsf
qp_solver_iter_max = 500;
nlp_solver_max_iter = 1000;

%% model dynamics
model = pushing_model();
nx = model.nx;
nu = model.nu;
%% model to create the solver
ocp_model = acados_ocp_model();
model_name = 'pushing';

%% acados ocp model
ocp_model.set('name', model_name);
ocp_model.set('T', T);

% symbolics
ocp_model.set('sym_x', model.sym_x);
ocp_model.set('sym_u', model.sym_u);
ocp_model.set('sym_xdot', model.sym_xdot);

% cost
ocp_model.set('cost_expr_ext_cost', model.expr_ext_cost);
ocp_model.set('cost_expr_ext_cost_e', model.expr_ext_cost_f);

% dynamics
if (strcmp(sim_method, 'erk'))
    ocp_model.set('dyn_type', 'explicit');
    ocp_model.set('dyn_expr_f', model.expr_f_expl);
else % irk irk_gnsf
    ocp_model.set('dyn_type', 'implicit');
    ocp_model.set('dyn_expr_f', model.expr_f_impl);
end

% constraints
ocp_model.set('constr_type', 'auto');
ocp_model.set('constr_expr_h', model.expr_h);
H_min =[-100;-100;-2*pi;3*pi/4;0;-100;0;0;0;0];
H_max = [100;100;2*pi;5*pi/4;100;100;5;5;9999;9999];
ocp_model.set('constr_lh', H_min); % lower bound on h
ocp_model.set('constr_uh', H_max);  % upper bound on h
ocp_model.set('sym_p', model.sym_p);
ocp_model.set('constr_x0', x0);

%% acados ocp set opts
ocp_opts = acados_ocp_opts();
ocp_opts.set('param_scheme_N', N);
ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('sim_method', sim_method);
ocp_opts.set('nlp_solver_max_iter', nlp_solver_max_iter);
ocp_opts.set('qp_solver', qp_solver);
ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
ocp_opts.set('qp_solver_iter_max', qp_solver_iter_max);
ocp_opts.set('parameter_values',[Xstar;5;10;10;10;1;1;1;1;0]);
ocp = acados_ocp(ocp_model, ocp_opts);

x_traj_init = zeros(nx, N+1);
u_traj_init = zeros(nu, N);

%% call ocp solver
% update initial state
ocp.set('constr_x0', x0);

% set trajectory initialization
ocp.set('init_x', x_traj_init);
ocp.set('init_u', u_traj_init);
ocp.set('init_pi', zeros(nx, N));
ocp.set('init_pi', zeros(nx, N));



% solve
ocp.solve();
% get solution
utraj = ocp.get('u');
xtraj = ocp.get('x');

status = ocp.get('status'); % 0 - success
ocp.print('stat')

%% Plots
ts = linspace(0, T, N+1);
figure; hold on;
States = {'x', 'y', 'orientation', 'pusher angle'};
for i=1:length(States)
    subplot(length(States), 1, i);
    plot(ts, xtraj(i,:)); grid on;
    ylabel(States{i});
    xlabel('t [s]')
end

ts = linspace(0, T, N);
figure; hold on;
States = {'normal force', 'tangential force', 'phi dot plus', 'phi dot minus'};
for i=1:length(States)
    subplot(length(States), 1, i);
    plot(ts, utraj(i,:)); grid on;
    ylabel(States{i});
    xlabel('t [s]')
end

%% go embedded
% to generate templated C code
%ocp.generate_c_code;


