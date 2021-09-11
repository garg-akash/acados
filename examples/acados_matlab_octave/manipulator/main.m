clear
clc
close all

% check that env.sh has been run
env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
	error('env.sh has not been sourced! Before executing this example, run: source env.sh');
end

addpath('./kinematics/')
addpath('./kinematics/screw/screws/') %required for ad()
addpath('./kinematics/screw/util/') %required for isequalf()

%% robot defination using Petercorke
global d1 d3 d5 d7 use_tau_f LWR
global robot
d1=0.0;     % distance from base frame to second joint frame
d3=0.4;     % distance from second joint frame to fourth joint frame
d5=0.39;    % distance from fourth joint frame to sixth joint frame
d7=0.078;
use_tau_f=false;

dq_init(:,1) = zeros(6,1);

clear L
L(1) = Revolute('d', 0, 'a', 0, 'alpha', pi/2, ...
    'I', [1 1 1*0.5^2/12], 'r', [-0.25 0 0], 'm', 1);
L(2) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2, ...
    'I', [1 1 1*0.5^2/12], 'r', [-0.25 0 0], 'm', 1);
L(3) = Revolute('d', d3, 'a', 0, 'alpha', -pi/2, ...
    'I', [1 1 1*0.5^2/12], 'r', [-0.25 0 0], 'm', 1);
L(4) = Revolute('d', 0, 'a', 0, 'alpha', pi/2, ...
    'I', [1 1 1*0.5^2/12], 'r', [-0.25 0 0], 'm', 1);
L(5) = Revolute('d', d5, 'a', 0, 'alpha', pi/2, ...
    'I', [1 1 1*0.5^2/12], 'r', [-0.25 0 0], 'm', 1);
L(6) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2, ...
    'I', [1 1 1*0.5^2/12], 'r', [-0.25 0 0], 'm', 1);
L(7) = Revolute('d', d7, 'a', 0, 'alpha', 0, ...
    'I', [1 1 1*0.5^2/12], 'r', [-0.25 0 0], 'm', 1);

LWR = SerialLink(L, 'name', 'Kuka LWR');

LWR.qlim(1,1) = -170*pi/180;
LWR.qlim(1,2) = 170*pi/180;
LWR.qlim(2,1) = -120*pi/180;
LWR.qlim(2,2) = 120*pi/180;
LWR.qlim(3,1) = -170*pi/180;
LWR.qlim(3,2) = 170*pi/180;
LWR.qlim(4,1) = -120*pi/180;
LWR.qlim(4,2) = 120*pi/180;
LWR.qlim(5,1) = -170*pi/180;
LWR.qlim(5,2) = 170*pi/180;
LWR.qlim(6,1) = -120*pi/180;
LWR.qlim(6,2) = 120*pi/180;
LWR.qlim(7,1) = -170*pi/180;
LWR.qlim(7,2) = 170*pi/180;

%% 
load_init_params

o.reset();

disp('Calculating inverse kinematics')
load ref2.mat

Fc_hat = blkdiag(o.contacts(1).x_hat_, o.contacts(2).x_hat_, o.contacts(3).x_hat_, o.contacts(4).x_hat_);

%% 
q_log = []; dq_log = [];
Je_current = LWR.jacob0(q_ref(:,1)');
M_m = LWR.inertia(q_ref(:,1)'); 
M_inv = inv(M_m);
C_m = LWR.coriolis(q_ref(:,1)',dq_ref(:,1)')*dq_ref(:,1);
N_m = LWR.gravload(q_ref(:,1)')'; %outputs col vector...take transpose
param_log = [M_inv(:); C_m(:); N_m(:)]';
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
% manipulator mpc 
model = manipulator_model();
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
Q = blkdiag(0*eye(7),10*eye(7),1e-4*eye(7));
R = 1e-10*eye(nu);
W = blkdiag(Q, R); % weight matrix in lagrange term
W_e = Q; % weight matrix in mayer term
yref = zeros(ny, 1); % output reference in lagrange term
yref_e = zeros(ny_e, 1); % output reference in mayer term
% constraints
x0 = [N_m;q_ref(:,1);zeros(7,1)];
Jbx = zeros(nbx, nx); for ii=1:nbx Jbx(ii,ii)=1.0; end
lbx = [-2000*ones(7,1);deg2rad(-170);deg2rad(-120);deg2rad(-170); ...
       deg2rad(-120);deg2rad(-170);deg2rad(-120);deg2rad(-170);-100*ones(7,1)];
ubx = -lbx;
Jbu = zeros(nbu, nu); for ii=1:nbu Jbu(ii,ii)=1.0; end
lbu = -1000*ones(nu, 1);
ubu = 1000*ones(nu, 1);

%% acados ocp model
ocp_model = acados_ocp_model();
% ocp_model.set('T', T);
ocp_model.set('T', dt*ocp_N);
% symbolics
ocp_model.set('sym_x', model.sym_x);
ocp_model.set('sym_u', model.sym_u);
ocp_model.set('sym_p', model.sym_p);
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
        ocp.set('p', [M_inv(:); C_m(:); N_m(:)], k);
    end
    
    for k = 0:ocp_N-1 %new - set the reference to track
        yref = ref(ocp_N, k+ii, q_ref, dq_ref);
        yref = [yref;zeros(7,1)];
        ocp.set('cost_y_ref', yref, k);
    end
    yref_e = ref(ocp_N, k+ii, q_ref, dq_ref);
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
    sim.set('p', [M_inv(:); C_m(:); N_m(:)]);

	% simulate state
	sim.solve();

	% get new state
	x_sim(:,ii+1) = sim.get('xn');
    
    % update dynamic matrices
    M_m = LWR.inertia(x_sim(8:14,ii+1)');
    M_inv = inv(M_m);
    C_m = LWR.coriolis(x_sim(8:14,ii+1)',x_sim(15:21,ii+1)')*x_sim(15:21,ii+1);
    N_m = LWR.gravload(x_sim(8:14,ii+1)')'; %outputs col vector...take transpose   
    
    param_log = [param_log; [M_inv(:); C_m(:); N_m(:)]'];
end

avg_time_solve = toc/n_sim

status = ocp.get('status');

if status==0
	fprintf('\nsuccess!\n\n');
else
	fprintf('\nsolution failed!\n\n');
end

%% function definition
function y = ref(k,instant,q_init,dq_init)
    global LWR
    if(instant<=size(q_init,2)-k)
        y = [];
        y = [y; LWR.gravload(q_init(:,instant)')';q_init(:,instant);dq_init(:,instant)];%zeros(16,1)];
    else
        y = [];
        y = [y; LWR.gravload(q_init(:,end)')';q_init(:,end);dq_init(:,end)];
    end
end