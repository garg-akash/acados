clear
clc
% close all

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
load ref.mat
% q_ref = [];
% dq_ref = [];
% % q_new = q_initial;
% q_new = deg2rad([-45;-30;0;60;0;90;0]);
% for i = 1:size(p_ref,1)
%     R_wb = rotz(o_wb(i,1))*roty(o_wb(i,2))*rotx(o_wb(i,3));
%     T_wb = [R_wb,p_ref(i,:)';0 0 0 1];
%     T_we = T_wb/T_eb;
%     
% %     Je_current = computeJe(d3,d5,d7,q_new_);
% %     dq_new_ = Je_current\[pd_ref(i,:)';w_ref(:,i)]; %%use full jacobain
% %     q_new_ = q_new_ + dq_new_*dt;
% %     q_log = [q_log q_new_];
%     dq_new = pinv(LWR.jacob0(q_new))*[pd_ref(i,:)';w_ref(:,i)];
%     q_new = LWR.ikcon(T_we,q_new);
% 
%     q_ref = [q_ref q_new'];
%     dq_ref = [dq_ref dq_new];
% end

Fc_hat = blkdiag(o.contacts(1).x_hat_, o.contacts(2).x_hat_, o.contacts(3).x_hat_, o.contacts(4).x_hat_);
R_eb = T_eb(1:3,1:3);
Ad_eb = [[R_eb,skew(T_eb(1:3,4))*R_eb];[zeros(3),R_eb]];
%% 
q_log = []; dq_log = [];
Te = LWR.fkine(q_ref(:,1)');
Je_current = LWR.jacob0(q_ref(:,1)');
T_wb = Te.T*T_eb;
M_m = LWR.inertia(q_ref(:,1)'); 
M_inv = inv(M_m);
C_m = LWR.coriolis(q_ref(:,1)',dq_ref(:,1)')*dq_ref(:,1);
N_m = LWR.gravload(q_ref(:,1)')'; %outputs col vector...take transpose

Jb = Ad_eb\(blkdiag(Te.R',Te.R')*Je_current);
Jb_dot = zeros(6,7);

o.computeN(Te.R);
C_o = blkdiag(skew(zyx2R(o_ref(1,:))'*w_ref(:,1))*o.Mb(1),skew(zyx2R(o_ref(1,:))'*w_ref(:,1))*o.Mb(4:6,4:6));

M_tilde = M_m + Jb'*o.Mb*Jb;
M_tilde_inv = inv(M_tilde);
% C_tilde = C_m + Jb'*o.Mb*Jb_dot + Jb'*C_o*Jb;
N_tilde = N_m + Jb'*o.Nb;

%% handy arguments
compile_interface = 'auto';
codgen_model = 'true';
% simulation
sim_method = 'erk';
sim_sens_forw = 'false';
sim_num_stages = 4;
sim_num_steps = 1;
% ocp
ocp_N = 30;
ocp_nlp_solver = 'sqp';
% ocp_nlp_solver = 'sqp_rti';
ocp_nlp_solver_max_iter = 100; % useful when choosing 'sqp'
ocp_qp_solver = 'partial_condensing_hpipm';
%ocp_qp_solver = 'full_condensing_hpipm';
ocp_qp_solver_cond_N = 5;
ocp_sim_method = 'erk';
ocp_sim_method_num_stages = 4;
ocp_sim_method_num_steps = 1;
ocp_cost_type = 'linear_ls';
%ocp_cost_type = 'nonlinear_ls';
%ocp_cost_type = 'ext_cost';
regularize_method = 'no_regularize';
%regularize_method = 'project';
%regularize_method = 'project_reduc_hess';
%regularize_method = 'mirror';
%regularize_method = 'convexify';
ocp_levenberg_marquardt = 0.0;
ocp_nlp_solver_warm_start_first_qp = 1;
ocp_qp_solver_warm_start = 1;
ocp_nlp_solver_exact_hessian = 'false';
ocp_nlp_solver_ext_qp_res = 1;
ocp_qp_solver_ric_alg = 0;

%% setup problem
LAMBDA_CONST = 1;
% manipulator mpc 
model = manipulator_object_model(o.G,LAMBDA_CONST);
% dims
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
Q = blkdiag(1e-6*eye(7),1e0*eye(7),1e-1*eye(7));
R = 1e-7*eye(nu);
% Q = blkdiag(1e-8,1e-8,1e-6,1e-6,1e-6*eye(3),...
%     1e-2,1e-2,1e0,1e0,1e0*eye(3),...
%     1e-5,1e-5,1e-3,1e-3,1e-3*eye(3));
% R = 1e-8*eye(nu);
W = blkdiag(Q, R); % weight matrix in lagrange term
W_e = Q; % weight matrix in mayer term
yref = zeros(ny, 1); % output reference in lagrange term
yref_e = zeros(ny_e, 1); % output reference in mayer term
% constraints
x0 = [N_tilde;q_ref(:,1);zeros(7,1)];
Jbx = zeros(nbx, nx); for ii=1:nbx Jbx(ii,ii)=1.0; end
lbx = [-176;-176;-110;-110;-110;-40;-40;...
        deg2rad(-170);deg2rad(-120);deg2rad(-170); ...
        deg2rad(-120);deg2rad(-170);deg2rad(-120);deg2rad(-170);...
        deg2rad(-98);deg2rad(-98);deg2rad(-100); ...
        deg2rad(-98);deg2rad(-140);deg2rad(-180);deg2rad(-180)];
% lbx = [-500;-500;-500;-500;-500;-500;-500;...
%         deg2rad(-170);deg2rad(-170);deg2rad(-170); ...
%         deg2rad(-170);deg2rad(-170);deg2rad(-170);deg2rad(-170);...
%         deg2rad(-180);deg2rad(-180);deg2rad(-180); ...
%         deg2rad(-180);deg2rad(-180);deg2rad(-180);deg2rad(-180)];
ubx = -lbx;
% % for Rodyman
% lbx = [-2000*ones(7,1);deg2rad(-160);deg2rad(-130);deg2rad(-60); ...
%        deg2rad(-135);deg2rad(-150);deg2rad(-120);deg2rad(-150);-2*ones(7,1)];
% ubx = [2000*ones(7,1);deg2rad(160);deg2rad(170);deg2rad(240); ...
%        deg2rad(135);deg2rad(150);deg2rad(120);deg2rad(150);2*ones(7,1)];
Jbu = zeros(nbu, nu); for ii=1:nbu Jbu(ii,ii)=1.0; end
lbu = -1e4*ones(nu, 1);
ubu = 1e4*ones(nu, 1);
if (LAMBDA_CONST)
    lbh = -1e-2*ones(16,1);%zeros(16,1); % bounds on lambda constraint
    ubh = 1e2*ones(16,1);
end
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
if (LAMBDA_CONST)
    ocp_model.set('constr_expr_h', model.expr_h);
    ocp_model.set('constr_lh', lbh);
    ocp_model.set('constr_uh', ubh);
end

ocp_model.model_struct

%% acados ocp opts
ocp_opts = acados_ocp_opts();
ocp_opts.set('compile_interface', compile_interface);
ocp_opts.set('codgen_model', codgen_model);
ocp_opts.set('param_scheme_N', ocp_N);
ocp_opts.set('nlp_solver', ocp_nlp_solver);
if (strcmp(ocp_nlp_solver, 'sqp'))
	ocp_opts.set('nlp_solver_max_iter', ocp_nlp_solver_max_iter);
end
ocp_opts.set('qp_solver', ocp_qp_solver);
if (strcmp(ocp_qp_solver, 'partial_condensing_hpipm'))
	ocp_opts.set('qp_solver_cond_N', ocp_qp_solver_cond_N); %New horizon after partial condensing
    ocp_opts.set('qp_solver_ric_alg', ocp_qp_solver_ric_alg);
end
ocp_opts.set('levenberg_marquardt', ocp_levenberg_marquardt);
ocp_opts.set('sim_method', ocp_sim_method);
ocp_opts.set('sim_method_num_stages', ocp_sim_method_num_stages);
ocp_opts.set('sim_method_num_steps', ocp_sim_method_num_steps);
ocp_opts.set('regularize_method', regularize_method);
ocp_opts.set('nlp_solver_warm_start_first_qp', ocp_nlp_solver_warm_start_first_qp);
ocp_opts.set('qp_solver_warm_start', ocp_qp_solver_warm_start);
ocp_opts.set('nlp_solver_exact_hessian', ocp_nlp_solver_exact_hessian);
ocp_opts.set('nlp_solver_ext_qp_res', ocp_nlp_solver_ext_qp_res);

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
ocp.generate_c_code
%% closed loop simulation

n_sim = floor(T/dt);
x_sim = zeros(nx, n_sim+1);
x_sim(:,1) = x0;
u_sim = zeros(nu, n_sim);

% x_traj_init = zeros(nx, ocp_N+1);
x_traj_init = repmat(x0,1, ocp_N+1);
u_traj_init = zeros(nu, ocp_N);

% lambda_log = pinv(o.G*Fc_hat)*(o.Mb*(Jb*(x_sim(15:21,1)-zeros(7,1))/dt) + ...
%                 C_o*Jb*x_sim(15:21,ii+1) + o.Nb);
% lambda_log = pinv(o.G*Fc_hat)*(o.Mb*(Jb*(x_sim(15:21,1)-zeros(7,1))/dt) + ...
%                  o.Nb);
Fc_read = zeros(12,1); % to be corrected
Fb_read = o.G*Fc_read;
ddq = pinv(M_m)*(x_sim(1:7,1) -Jb'*Fb_read - C_m - N_m);
ddx = Jb*ddq + Jb_dot*x_sim(15:21,1);
lambda_log = pinv(o.G*Fc_hat)*(o.Mb*ddx + o.Nb);
tic;

for ii=1:n_sim
	% set x0
    fprintf('iter: %d\n', ii);
	ocp.set('constr_x0', x_sim(:,ii));

	% set trajectory initialization
	ocp.set('init_x', x_traj_init);
	ocp.set('init_u', u_traj_init);
    
    % set parameter
    for k=0:ocp_N-1
%         ocp.set('p', [M_tilde_inv(:); C_m(:); N_tilde(:); Jb(:); o.Mb(:); C_o(:); o.Nb(:); x_sim(15:21,ii)], k);
        ocp.set('p', [M_tilde(:); C_m(:); N_tilde(:); Jb(:); ...
            o.Mb(:); C_o(:); o.Nb(:); zeros(42,1); ...
            M_m(:); C_m(:); N_m(:); Fc_read], k);
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
    ocp.print('stat')
    % get cost value
    cost_val_ocp(ii) = ocp.get_cost();

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
    sim.set('p', [M_tilde(:); C_m(:); N_tilde(:); Jb(:);...
        o.Mb(:); C_o(:); o.Nb(:); zeros(42,1); ...
        M_m(:); C_m(:); N_m(:); Fc_read]);

	% simulate state
	sim.solve();

	% get new state
	x_sim(:,ii+1) = sim.get('xn');
    
    % update dynamic matrices
    Te = LWR.fkine(x_sim(8:14,ii+1)');
    T_wb = Te.T*T_eb;
    Je_current = LWR.jacob0(x_sim(8:14,ii+1)');
    
    M_m = LWR.inertia(x_sim(8:14,ii+1)');
    M_inv = inv(M_m);
    C_m = LWR.coriolis(x_sim(8:14,ii+1)',x_sim(15:21,ii+1)')*x_sim(15:21,ii+1);
    N_m = LWR.gravload(x_sim(8:14,ii+1)')'; %outputs col vector...take transpose   
    
    Jb = Ad_eb\(blkdiag(Te.R',Te.R')*Je_current);
    
    o.computeN(Te.R);
    xdot = Jb*x_sim(15:21,ii+1);
    C_o = blkdiag(skew(xdot(4:6))*o.Mb(1),skew(xdot(4:6))*o.Mb(4:6,4:6));
    
    M_tilde = M_m + Jb'*o.Mb*Jb;
    M_tilde_inv = inv(M_tilde);
    C_tilde = C_m + Jb'*o.Mb*Jb_dot + Jb'*C_o*Jb;
    N_tilde = N_m + Jb'*o.Nb;
    
%     lambda_log = [lambda_log pinv(o.G*Fc_hat)*(o.Mb*(Jb*(x_sim(15:21,ii+1)-...
%         x_sim(15:21,ii))/dt) + o.Nb)];
    Fb_read = o.G*Fc_read;
    ddq = pinv(M_m)*(x_sim(1:7,ii) - Jb'*Fb_read - C_m - N_m);
    ddx = Jb*ddq + Jb_dot*x_sim(15:21,ii);
    lambda_log = [lambda_log pinv(o.G*Fc_hat)*(o.Mb*ddx + o.Nb)];
    
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