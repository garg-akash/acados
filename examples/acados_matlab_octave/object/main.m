clear all

% check that env.sh has been run
env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
	error('env.sh has not been sourced! Before executing this example, run: source env.sh');
end

addpath('./kinematics/')
addpath('./kinematics/screw/screws/') %required for ad()
addpath('./kinematics/screw/util/') %required for isequalf()

load_init_params

o.reset();
Fc_hat = blkdiag(o.contacts(1).x_hat_, o.contacts(2).x_hat_, o.contacts(3).x_hat_, o.contacts(4).x_hat_);
M_inv = inv(o.Mb);
C_ = zeros(6,6);
N_ = [0;0;4.9;0;0;0];
dyn = [M_inv(:); C_(:); N_(:)];

%% handy arguments
compile_interface = 'auto';
codgen_model = 'true';
% simulation
sim_method = 'erk';
sim_sens_forw = 'false';
sim_num_stages = 4;
sim_num_steps = 1;
% ocp
ocp_N = 20;
% ocp_nlp_solver = 'sqp';
ocp_nlp_solver = 'sqp_rti';
ocp_qp_solver = 'partial_condensing_hpipm';
%ocp_qp_solver = 'full_condensing_hpipm';
ocp_qp_solver_cond_N = 5;
%ocp_sim_method = 'erk';
ocp_sim_method = 'erk';
ocp_sim_method_num_stages = 4;
ocp_sim_method_num_steps = 1;
ocp_cost_type = 'linear_ls';
%ocp_cost_type = 'nonlinear_ls';
%ocp_cost_type = 'ext_cost';

%% setup problem
% object mpc 
model = object_model(o.G, Fc_hat);
% dims
% T = 2.5; % horizon length time %already defined in load_init_params
% h = 0.01;
nx = model.nx; % number of states
nu = model.nu; % number of inputs
ny = nx+nu; % number of outputs in lagrange term
ny_e = nx; % number of outputs in mayer term
nbx = nx; % number of state bounds
nbu = nu; % number of input bounds
% cost
Vx = zeros(ny, nx); for ii=1:nx Vx(ii,ii)=1.0; end % state-to-output matrix in lagrange term
Vu = zeros(ny, nu); for ii=1:nu Vu(nx+ii,ii)=1.0; end % input-to-output matrix in lagrange term
Vx_e = zeros(ny_e, nx); for ii=1:nx Vx_e(ii,ii)=1.0; end % state-to-output matrix in mayer term
Q = blkdiag(10,10,10,10,10,10,1e-2,1e-2,1e-2,1e-2,1e-2,1e-2,0,1e-4*eye(16));
R = 1e-4*eye(nu);
W = blkdiag(Q, R); % weight matrix in lagrange term
W_e = Q; % weight matrix in mayer term
yref = zeros(ny, 1); % output reference in lagrange term
yref_e = zeros(ny_e, 1); % output reference in mayer term
% constraints
x0 = [p_ref(1,:)';o_ref(1,:)';(zyx2R(o_ref(1,:))'*pd_ref(1,:)');(zyx2R(o_ref(1,:))'*w_ref(1:3,1));9.8;0.3*ones(16,1)];
Jbx = zeros(nbx, nx); for ii=1:nbx Jbx(ii,ii)=1.0; end
lbx = [-2;-2;-2;-pi;-pi/2;-pi;-2;-2;-2;-pi;-pi;-pi;9.8;zeros(16,1)];
ubx = [2;2;2;pi;pi/2;pi;2;2;2;pi;pi;pi;9.8;20*ones(16,1)];
Jbu = zeros(nbu, nu); for ii=1:nbu Jbu(ii,ii)=1.0; end
lbu = -10000*ones(nu, 1);
ubu = 10000*ones(nu, 1);

%% acados ocp model
ocp_model = acados_ocp_model();
% ocp_model.set('T', T);
ocp_model.set('T', dt*ocp_N);
% symbolics
ocp_model.set('sym_x', model.sym_x);
ocp_model.set('sym_u', model.sym_u);
ocp_model.set('sym_p', model.sym_p);
% ocp_model.set('sym_xdot', model.sym_xdot);
% cost
ocp_model.set('cost_type', ocp_cost_type);
ocp_model.set('cost_type_e', ocp_cost_type);
ocp_model.set('cost_Vu', Vu);
ocp_model.set('cost_Vx', Vx);
ocp_model.set('cost_Vx_e', Vx_e);
ocp_model.set('cost_W', W);
ocp_model.set('cost_W_e', W_e);
ocp_model.set('cost_y_ref', yref);
ocp_model.set('cost_y_ref_e', yref_e);
% dynamics
ocp_model.set('dyn_type', 'explicit');
ocp_model.set('dyn_expr_f', model.expr_f_expl);
% constraints
ocp_model.set('constr_x0', x0);
ocp_model.set('constr_Jbx', Jbx);
ocp_model.set('constr_lbx', lbx);
ocp_model.set('constr_ubx', ubx);
ocp_model.set('constr_Jbu', Jbu);
ocp_model.set('constr_lbu', lbu);
ocp_model.set('constr_ubu', ubu);

ocp_model.model_struct

%% acados ocp opts
ocp_opts = acados_ocp_opts();
ocp_opts.set('compile_interface', compile_interface);
ocp_opts.set('codgen_model', codgen_model);
ocp_opts.set('param_scheme_N', ocp_N);
ocp_opts.set('nlp_solver', ocp_nlp_solver);
ocp_opts.set('qp_solver', ocp_qp_solver);
if (strcmp(ocp_qp_solver, 'partial_condensing_hpipm'))
	ocp_opts.set('qp_solver_cond_N', ocp_qp_solver_cond_N);
end
ocp_opts.set('sim_method', ocp_sim_method);
ocp_opts.set('sim_method_num_stages', ocp_sim_method_num_stages);
ocp_opts.set('sim_method_num_steps', ocp_sim_method_num_steps);
ocp_opts.set('regularize_method', 'no_regularize');

ocp_opts.opts_struct

%% acados ocp
% create ocp
ocp = acados_ocp(ocp_model, ocp_opts);
ocp

%% acados sim model
sim_model = acados_sim_model();
% symbolics
sim_model.set('sym_x', model.sym_x);
sim_model.set('sym_u', model.sym_u);
sim_model.set('sym_p', model.sym_p);
% sim_model.set('sym_xdot', model.sym_xdot);
% model
% sim_model.set('T', T/ocp_N);
sim_model.set('T', dt);
sim_model.set('dyn_type', 'explicit');
sim_model.set('dyn_expr_f', model.expr_f_expl);

sim_model.model_struct

%% acados sim opts
sim_opts = acados_sim_opts();
sim_opts.set('compile_interface', compile_interface);
sim_opts.set('codgen_model', codgen_model);
sim_opts.set('num_stages', sim_num_stages);
sim_opts.set('num_steps', sim_num_steps);
sim_opts.set('method', sim_method);
sim_opts.set('sens_forw', sim_sens_forw);

sim_opts.opts_struct

%% acados sim
% create sim
sim = acados_sim(sim_model, sim_opts);
sim
% sim.C_sim
% sim.C_sim_ext_fun

%% closed loop simulation

n_sim = floor(T/dt);
x_sim = zeros(nx, n_sim+1);
x_sim(:,1) = x0;
u_sim = zeros(nu, n_sim);

% x_traj_init = zeros(nx, ocp_N+1);
x_traj_init = repmat(x0,1, ocp_N+1);
u_traj_init = zeros(nu, ocp_N);

tic;

for ii=1:n_sim

	% set x0
	ocp.set('constr_x0', x_sim(:,ii));

	% set trajectory initialization
	ocp.set('init_x', x_traj_init);
	ocp.set('init_u', u_traj_init);
    
    % set parameter
    for k=0:ocp_N-1
        ocp.set('p', [M_inv(:); C_(:); N_(:)], k);
    end
    
    for k = 0:ocp_N-1 %new - set the reference to track
        yref = ref(zyx2R(x_sim(4:6,ii)), p_ref, pd_ref, ocp_N, k+ii, o_ref, w_ref);
        yref = [yref;zeros(16,1)];
        ocp.set('cost_y_ref', yref, k);
    end
    yref_e = ref(zyx2R(x_sim(4:6,ii)), p_ref, pd_ref, ocp_N, k+ii, o_ref, w_ref);
    ocp.set('cost_y_ref_e', yref_e, ocp_N);
    
	% solve OCP
	ocp.solve();

	% get solution
	%x_traj = ocp.get('x');
	%u_traj = ocp.get('u');
	u_sim(:,ii) = ocp.get('u', 0);
    x_traj_init = ocp.get('x');
    x_traj_init = [x_traj_init(:,2:end), x_traj_init(:,end)];
    u_traj_init = ocp.get('u');
    u_traj_init = [u_traj_init(:,2:end), u_traj_init(:,end)];

	% set initial state of sim
	sim.set('x', x_sim(:,ii));
	% set input in sim
	sim.set('u', u_sim(:,ii));
    % set parameter
    sim.set('p', [M_inv(:); C_(:); N_(:)]);

	% simulate state
	sim.solve();

	% get new state
	x_sim(:,ii+1) = sim.get('xn');
end

avg_time_solve = toc/n_sim

status = ocp.get('status');

if status==0
	fprintf('\nsuccess!\n\n');
else
	fprintf('\nsolution failed!\n\n');
end

%% MPC outputs velocity in body frame, rotate it to plot in W frame
for i = 1:size(x_sim,2)
pd_op(:,i) = zyx2R(x_sim(4:6,i))*x_sim(7:9,i); 
wd_op(:,i) = zyx2R(x_sim(4:6,i))*x_sim(10:12,i); 
end

%% function definition
function y = ref(R,p,pd,k,instant,ang,wd)
    if(instant<=size(p,1)-k)
        y = [];
        y = [y; [p(instant,:)'; ang(instant,:)']; [R'*pd(instant,:)'; R'*wd(:,instant)];9.8;0.3*ones(16,1)];
    else
        y = [];
        y = [y; [p(size(p,1),:)'; ang(size(p,1),:)']; [R'*pd(size(p,1),:)'; R'*wd(:,size(p,1))];9.8;0.3*ones(16,1)];
    end
end