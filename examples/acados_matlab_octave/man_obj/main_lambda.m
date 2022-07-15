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
addpath('/home/akash/Documents/PeterCorkeToolbox/common/')
addpath('/home/akash/Documents/PeterCorkeToolbox/rtb/')
addpath('/home/akash/Documents/PeterCorkeToolbox/smtb/')
%% robot defination using Petercorke
global d1 d3 d5 d7 use_tau_f LWR
global robot
d1=0.0;     % distance from base frame to second joint frame
d3=0.4;     % distance from second joint frame to fourth joint frame
d5=0.39;    % distance from fourth joint frame to sixth joint frame
d7=0.078;
use_tau_f=false;

% dq_init(:,1) = zeros(6,1);

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
% load 15refData.mat
q_ref = [];
dq_ref = [];
% q_new = q_initial;
q_new = deg2rad([-45;-30;0;60;0;90;0]);
for i = 1:size(p_ref,1)
    R_wb = rotz(o_wb(i,1))*roty(o_wb(i,2))*rotx(o_wb(i,3));
    T_wb = [R_wb,p_ref(i,:)';0 0 0 1];
    T_we = T_wb/T_eb;
    
%     Je_current = computeJe(d3,d5,d7,q_new_);
%     dq_new_ = Je_current\[pd_ref(i,:)';w_ref(:,i)]; %%use full jacobain
%     q_new_ = q_new_ + dq_new_*dt;
%     q_log = [q_log q_new_];
    dq_new = pinv(LWR.jacob0(q_new))*[pd_ref(i,:)';w_ref(:,i)]; %(v,w) of ee are coming out to be same as that of obj
    q_new = LWR.ikcon(T_we,q_new);

    q_ref = [q_ref q_new'];
    dq_ref = [dq_ref dq_new];
end

Fc_hat = blkdiag(o.contacts(1).x_hat_, o.contacts(2).x_hat_, o.contacts(3).x_hat_, o.contacts(4).x_hat_);
R_eb = T_eb(1:3,1:3);
Ad_eb = [[R_eb,skew(T_eb(1:3,4))*R_eb];[zeros(3),R_eb]];
%% 
q_log = []; dq_log = [];
tau_log = []; dtau_log = [];
p_log = []; phi_log = [];
p_ref_log = []; phi_ref_log = [];
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
ocp_N = 10;
ocp_nlp_solver = 'sqp_rti';
% ocp_nlp_solver = 'sqp';
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
ocp_levenberg_marquardt = 1e-3;

%% setup problem
% manipulator mpc 
model = manipulator_object_model_lambda(o.G);
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
Q = blkdiag(1e-2*eye(7), 1e7*eye(7), 1e5*eye(7), 1e0*eye(16));
R = 1e-4*eye(nu);
% Q = blkdiag(1e-8,1e-8,1e-6,1e-6,1e-6*eye(3),...
%     1e-2,1e-2,1e0,1e0,1e0*eye(3),...
%     1e-5,1e-5,1e-3,1e-3,1e-3*eye(3));
% R = 1e-8*eye(nu);
W = blkdiag(Q, R); % weight matrix in lagrange term
W_e = 1e-3*Q; % weight matrix in mayer term
yref = zeros(ny, 1); % output reference in lagrange term
yref_e = zeros(ny_e, 1); % output reference in mayer term
% constraints

x0 = [N_tilde;q_ref(:,1);zeros(7,1); pinv(o.G*Fc_hat)*o.Nb];
Jbx = zeros(nbx, nx); for ii=1:nbx Jbx(ii,ii)=1.0; end
lbx = [-176;-176;-110;-110;-110;-40;-40;...
        deg2rad(-170);deg2rad(-120);deg2rad(-170); ...
        deg2rad(-120);deg2rad(-170);deg2rad(-120);deg2rad(-170);...
        deg2rad(-98);deg2rad(-98);deg2rad(-100); ...
        deg2rad(-98);deg2rad(-140);deg2rad(-180);deg2rad(-180);
        1e-3*ones(16,1)];
%         -2*ones(16,1)]; %in case of no lambda constraint
ubx = -lbx;
ubx(22:37) = 2;

Jbu = zeros(nbu, nu); for ii=1:nbu Jbu(ii,ii)=1.0; end
lbu = -200*ones(nu, 1); %1000 in case of no tau_dot constraint
ubu = 200*ones(nu, 1);

%% acados ocp model
ocp_model = acados_ocp_model();
% ocp_model.set('T', T);
ocp_model.set('T', ocp_N*dt);
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
	ocp_opts.set('qp_solver_cond_N', ocp_qp_solver_cond_N); %New horizon after partial condensing
end
ocp_opts.set('levenberg_marquardt', ocp_levenberg_marquardt);
ocp_opts.set('sim_method', ocp_sim_method);
ocp_opts.set('sim_method_num_stages', ocp_sim_method_num_stages);
ocp_opts.set('sim_method_num_steps', ocp_sim_method_num_steps);
ocp_opts.set('regularize_method', 'convexify');
% ocp_opts.set('qp_solver_warm_start', 1);
% ocp_opts.set('print_level', 0);

ocp_opts.opts_struct

%% acados ocp
% create ocp
ocp = acados_ocp(ocp_model, ocp_opts);
ocp
ocp.generate_c_code

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

n_sim = floor(2*T/dt);
x_sim = zeros(nx, n_sim+1);
x_sim(:,1) = x0;
u_sim = zeros(nu, n_sim);

% x_traj_init = zeros(nx, ocp_N+1);
x_traj_init = repmat(x0, 1, ocp_N+1);
u_traj_init = zeros(nu, ocp_N);

% lambda_log = pinv(o.G*Fc_hat)*(o.Mb*(Jb*(x_sim(15:21,1)-zeros(7,1))/dt) + ...
%                 C_o*Jb*x_sim(15:21,ii+1) + o.Nb);
% lambda_log = pinv(o.G*Fc_hat)*(o.Mb*(Jb*(x_sim(15:21,1)-zeros(7,1))/dt) + ...
%                  o.Nb);

Fb_read = o.Nb;
Fc_read = pinv(o.G)*Fb_read; 
ddq = zeros(7,1);
ddx = zeros(6,1);
lambda_log = pinv(o.G*Fc_hat)*Fb_read;
lambda_ref = pinv(o.G*Fc_hat)*o.Nb;
La_read = lambda_ref;
tic;

% set trajectory initialization
ocp.set('init_x', x_traj_init);
ocp.set('init_u', u_traj_init);   

for ii=1:n_sim
	% set x0
    fprintf('iter: %d\n', ii);
    x0 = x_sim(:,ii);
    x0(22:37) = La_read;
	ocp.set('constr_x0', x0);

    % set parameter
    for k=0:ocp_N-1
%         ocp.set('p', [M_tilde_inv(:); C_m(:); N_tilde(:); Jb(:); o.Mb(:); C_o(:); o.Nb(:); x_sim(15:21,ii)], k);
        ocp.set('p', [M_tilde(:); C_m(:); N_tilde(:); Jb(:); ...
            o.Mb(:); C_o(:); o.Nb(:); zeros(42,1); ...
            M_m(:); C_m(:); N_m(:); Fb_read; mu], k);
    end
    
    % compute reference
    for k = 0:ocp_N-1 %new - set the reference to track
        yref = [ref(ocp_N, k+ii, q_ref, dq_ref); lambda_ref; zeros(7,1);];
        ocp.set('cost_y_ref', yref, k);
    end
    yref_e = [ref(ocp_N, k+ii+1, q_ref, dq_ref); lambda_ref];
    ocp.set('cost_y_ref_e', yref_e, ocp_N);
    
	% solve OCP
	ocp.solve();
    
    % get cost value
    cost_val_ocp(ii) = ocp.get_cost();

	% get solution
	%x_traj = ocp.get('x');
	%u_traj = ocp.get('u');
	u0 = ocp.get('u', 0);

	% set initial state of sim
	sim.set('x', x0);
    % set input in sim
	sim.set('u', u0);
    % set parameter
    sim.set('p', [M_tilde(:); C_m(:); N_tilde(:); Jb(:);...
        o.Mb(:); C_o(:); o.Nb(:); zeros(42,1); ...
        M_m(:); C_m(:); N_m(:); Fb_read; mu]);

    % solve
    sim_status = sim.solve();
    if sim_status ~= 0
        disp(['acados integrator returned error status ', num2str(sim_status)])
    end

	% get new state
	x_sim(:,ii+1) = sim.get('xn');
    u_sim(:,ii) = u0;
    
%     if (ii >= 100 && ii <= 150) % disturbance torque
%         x_sim(1:7,ii+1) = x_sim(1:7,ii+1) + Jb'*[20*sin((ii - 100)/50*pi);0; 0; 0; 0; 0];
%     end
    
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
%     C_tilde = C_m + Jb'*o.Mb*Jb_dot + Jb'*C_o*Jb; %(wrong...adding vector and matrix)
    C_tilde = C_m;
    N_tilde = N_m + Jb'*o.Nb;
    
%     lambda_log = [lambda_log pinv(o.G*Fc_hat)*(o.Mb*(Jb*(x_sim(15:21,ii+1)-...
%         x_sim(15:21,ii))/dt) + o.Nb)];

    %ddq = pinv(M_m)*(x_sim(1:7,ii) - Jb'*Fb_read - C_m - N_m);
    ddq = M_tilde_inv*(x_sim(1:7,ii+1) - C_tilde - N_tilde);
    ddx = Jb*ddq + Jb_dot*x_sim(15:21,ii+1);
    Fb_read = o.Mb*ddx + o.Nb;
    Fc_read = pinv(o.G)*Fb_read;
    La_read = pinv(Fc_hat)*Fc_read;

    % store logs
    p_log = [p_log, T_wb(1:3,4)];
    phi_log = [phi_log, rpy(T_wb(1:3,1:3))];
    lambda_log = [lambda_log La_read];
    dq_log = [dq_log, x_sim(15:21,ii+1)];
    q_log = [q_log, x_sim(8:14,ii+1)]; 
    tau_log = [tau_log, x_sim(1:7,ii+1)];
    dtau_log = [dtau_log, u_sim(1:7,ii)];
    
    status = ocp.get('status');

    if status==0
        fprintf('\nsuccess!\n\n');
    else
        fprintf('\nsolution failed with status %d!\n\n', status);
        break;
    end
    
end

avg_time_solve = toc/n_sim

%% PLOTS
% % p  
% figure
% for i = 1:3
% subplot(3,1,i)
% plot(p_log(i,:),'-','linewidth',2)
% grid on
% hold on
% plot(p(:,i)','--','linewidth',2)
% yl = strcat('$p_{', strcat(int2str(i), '}$'));
% xlabel('iteration','Interpreter','latex');ylabel(yl,'Interpreter','latex')
% set(gca, 'FontSize', 10)
% end
% 
% % phi 
% figure
% for i = 1:3
% subplot(3,1,i)
% plot(phi_log(4-i,:),'-','linewidth',2)
% grid on
% hold on
% plot(o_wb(:,i)','--','linewidth',2)
% yl = strcat('$\phi_{', strcat(int2str(i), '}$'));
% xlabel('iteration','Interpreter','latex');ylabel(yl,'Interpreter','latex')
% set(gca, 'FontSize', 10)
% end
%         
% % dq  
% figure
% for i = 1:7
% subplot(3,3,i)
% plot(dq_log(i,:),'-','linewidth',2)
% grid on
% hold on
% plot(dq_ref(i,1:size(dq_ref,2)),'--','linewidth',2)
% plot(repmat(lbx(14+i),size(dq_log,2)),'--r','linewidth',2) 
% plot(repmat(ubx(14+i),size(dq_log,2)),'--r','linewidth',2)
% yl = strcat('$\dot{q}_', strcat(int2str(i), '$'));
% xlabel('iteration','Interpreter','latex');ylabel(yl,'Interpreter','latex')
% set(gca, 'FontSize', 10)
% end
%         
% % q  
% figure
% for i = 1:7
% subplot(3,3,i)
% plot(q_log(i,:),'-','linewidth',2)
% grid on
% hold on
% plot(q_ref(i,1:size(q_ref,2)),'--','linewidth',2)
% plot(repmat(lbx(7+i),size(q_log,2)),'--r','linewidth',2) 
% plot(repmat(ubx(7+i),size(q_log,2)),'--r','linewidth',2)
% yl = strcat('$q_', strcat(int2str(i), '$'));
% xlabel('iteration','Interpreter','latex');ylabel(yl,'Interpreter','latex')
% set(gca, 'FontSize', 10)
% end
% 
% % tau  
% figure
% for i = 1:7
% subplot(3,3,i)
% plot(tau_log(i,:),'-','linewidth',2)
% grid on
% hold on 
% plot(repmat(lbx(i),size(tau_log,2)),'--r','linewidth',2) 
% plot(repmat(ubx(i),size(tau_log,2)),'--r','linewidth',2)
% yl = strcat('$\tau_', strcat(int2str(i), '$'));
% xlabel('iteration','Interpreter','latex');ylabel(yl,'Interpreter','latex')
% set(gca, 'FontSize', 10)
% end
% 
% % dtau
% figure
% for i = 1:7
% subplot(3,3,i)
% plot(dtau_log(i,:),'-','linewidth',2)
% grid on
% hold on 
% plot(repmat(lbu(i),size(dtau_log,2)),'--r','linewidth',2) 
% plot(repmat(ubu(i),size(dtau_log,2)),'--r','linewidth',2)
% yl = strcat('$\dot{\tau}_', strcat(int2str(i), '$'));
% xlabel('iteration','Interpreter','latex');ylabel(yl,'Interpreter','latex')
% set(gca, 'FontSize', 10)
% end   
% 
% % lambda
% figure
% for i = 1:16
% subplot(4,4,i)
% plot(lambda_log(i,:),'-','linewidth',2)
% grid on
% hold on
% plot(repmat(lbx(21+i),size(lambda_log,2)),'--r','linewidth',2) 
% plot(repmat(ubx(21+i),size(lambda_log,2)),'--r','linewidth',2)
% yl = strcat('$\lambda_{', strcat(int2str(i), '}$'));
% xlabel('iteration','Interpreter','latex');ylabel(yl,'Interpreter','latex')
% set(gca, 'FontSize', 10)
% end
% 
% save('data_dist.mat', 'p_log', 'phi_log', 'lambda_log', 'dq_log', 'q_log', 'tau_log', 'dtau_log', 'cost_val_ocp', 'q_ref', 'dq_ref', 'p', 'o_wb', 'lbx', 'ubx', 'lbu', 'ubu')


%% function definition
function y = ref(k,instant,q_init,dq_init)
    global LWR
    if(instant<=size(q_init,2))
        y = [];
        y = [y; LWR.gravload(q_init(:,instant)')';q_init(:,instant);dq_init(:,instant)];%zeros(16,1)];
    else
        y = [];
        y = [y; LWR.gravload(q_init(:,end)')';q_init(:,end);dq_init(:,end)];
    end
end