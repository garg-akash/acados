clear all
clc
close all

%%
% check that env.sh has been run
env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
	error('env.sh has not been sourced! Before executing this example, run: source env.sh');
end

%%
show_plots = true;
save_plots = false;
show_video = true;
save_video = true;
video_title = "video_";

%%
addpath('./kinematics/')
addpath('./kinematics/screw/screws/') %required for ad()
addpath('./kinematics/screw/util/') %required for isequalf()
addpath('./ui/')
addpath('/home/akash/Documents/PeterCorkeToolbox/common/')
addpath('/home/akash/Documents/PeterCorkeToolbox/rtb/')
addpath('/home/akash/Documents/PeterCorkeToolbox/smtb/')

%% gains
params.p_gain = 10;
params.o_gain = 10;
params.v_gain = 1e-2;
params.w_gain = 1e2;
params.l_gain = 1e-4;
params.dl_gain = 1e-4;

%% bounds
params.p_bounds = [-100, 100];
params.oz_bounds = [-pi, pi];
params.oy_bounds = [-pi/2, pi/2];
params.ox_bounds = [-pi, pi];
params.v_bounds = [-1000, 1000];
params.w_bounds = [-1000, 1000];
params.l_bounds = [-0, 100];
params.dl_bounds = [-1e5, 1e5];
params.tau_bounds = [-100, 100];

%% limits
params.jnt_lim = [-170*pi/180, 170*pi/180; ...
    -120*pi/180, 120*pi/180; ...
    -170*pi/180, 170*pi/180; ...
    -120*pi/180, 120*pi/180; ...
    -170*pi/180, 170*pi/180; ...
    -120*pi/180, 120*pi/180; ...
    -170*pi/180, 170*pi/180];

%% simulation
params.sim_tim = 1.2; % simulation time [s]

%% robot defination using Petercorke's toolbox
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
LWR.qlim = params.jnt_lim;

%% Load object and trajectory
load_init_params;

%% Initialize figure
ui;

%% Define variables
global M_o % mass matrix of the object
M_o = o.Mb;

global R_eb
% tz_eb = 0.03; defined in load_init_params
% R_eb = eye(3); % rotation of body frame wrt ee
% T_eb = [R_eb,[0;0;tz_eb];0 0 0 1];
Ad_eb = ad(T_eb);

%% Compute Lambda references
% only used initial value, i.e. object at rest, consider removing
for i=1:size(tSteps,2)
    R_init = eye(3);
    dp = [eye(3), R_init'*p_ref(i,:)';
        0 0 0 1];
    dpd = [R_init'*pd_ref(i,:)'; 0; 0; 0];
    dpdd = [R_init'*pdd_ref(i,:)'; 0; 0; 0];
    
    Fbstar = o.inverseDynamics(dp, dpd, dpdd);
    lm_ref(i,:) = (pinv(o.G*o.Fc_hat)*Fbstar)'; % this is not the reference value (object state not updated in this loop!)
end

%% 
% o.reset();
% Fc_hat = blkdiag(o.contacts(1).x_hat_, o.contacts(2).x_hat_, o.contacts(3).x_hat_, o.contacts(4).x_hat_);
% R_eb = T_eb(1:3,1:3);
% Ad_eb = [[R_eb,skew(T_eb(1:3,4))*R_eb];[zeros(3),R_eb]];

% o.computeN(Te.R);
% N_o = o.Nb;

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
TAU_CONST = 1;

% object_manipulator mpc 
model = object_manipulator_model(o.G, o.Fc_hat, dt, TAU_CONST);

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

Q = zeros(nx,nx); % weighing matrix (states)
Q = blkdiag(eye(3)*params.p_gain, ... % position
    eye(3)*params.o_gain, ... % euler angles
    eye(3)*params.v_gain, ... % linear velocity
    eye(3)*params.w_gain, ... % angular velocity
    0, ...                    % gravity
    eye(16)*params.l_gain);   % lambda
R = params.dl_gain*eye(nu,nu); % weighing matrix (controls)

W = blkdiag(Q, R); % weight matrix in lagrange term
W_e = Q; % weight matrix in mayer term

yref = zeros(ny, 1); % output reference in lagrange term
yref_e = zeros(ny_e, 1); % output reference in mayer term

% constraints
x0 = [p_ref(1,:)';o_ref(1,:)';(zyx2R(o_ref(1,:))'*pd_ref(1,:)');(zyx2R(o_ref(1,:))'*w_ref(1:3,1));9.8;lm_ref(1,:)'];

Jbx = zeros(nbx, nx); for ii=1:nbx Jbx(ii,ii)=1.0; end
lbx = [ones(3,1)*params.p_bounds(1);...
    params.oz_bounds(1);params.oy_bounds(1);params.ox_bounds(1);...
    ones(3,1)*params.v_bounds(1);...
    ones(3,1)*params.w_bounds(1);...
    9.8;...
    ones(16,1)*params.l_bounds(1)];
ubx = [ones(3,1)*params.p_bounds(2);...
    params.oz_bounds(2);params.oy_bounds(2);params.ox_bounds(2);...
    ones(3,1)*params.v_bounds(2);...
    ones(3,1)*params.w_bounds(2);...
    9.8;...
    ones(16,1)*params.l_bounds(2)];
% lbx = [-100;-100;-100;-pi;-pi/2;-pi;-1000;-1000;-1000;-1000;-1000;-1000;9.8;-1e-10*ones(16,1)];
% ubx = [100;100;100;pi;pi/2;pi;1000;1000;1000;1000;1000;1000;9.8;2000*ones(16,1)];

Jbu = zeros(nbu, nu); for ii=1:nbu Jbu(ii,ii)=1.0; end
lbu = ones(nu,1)*params.dl_bounds(1);
ubu = ones(nu,1)*params.dl_bounds(2);

if (TAU_CONST)
    lbh = ones(7,1)*params.tau_bounds(1);%zeros(7,1); % bounds on tau constraint
    ubh = ones(7,1)*params.tau_bounds(2);
end

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
if (TAU_CONST)
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
	ocp_opts.set('qp_solver_cond_N', ocp_qp_solver_cond_N);
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

%% closed loop simulation

t0 = 0;
n_sim = floor(T/dt);
x_sim = zeros(nx, n_sim+1);
x_sim(:,1) = x0;
u_sim = zeros(nu, n_sim);

% x_traj_init = zeros(nx, ocp_N+1);
x_traj_init = repmat(x0,1, ocp_N+1);
u_traj_init = zeros(nu, ocp_N);

main_loop = tic;

logger.q = [];
logger.dq = [];
logger.tau = [];
logger.p = [];
logger.o = [];
logger.v = [];
logger.w = [];
logger.l = [];
logger.dl = [];
logger.e = [];
logger.cost = [];

q  = q_initial;
dq  = zeros(7,1);

Je = LWR.jacob0(q');
M_m = LWR.inertia(q');
C_m = LWR.coriolis(q',dq');
N_m = LWR.gravload(q')'; %outputs col vector...take transpose
R_wb_current = zyx2R(o_ref(1,:));
R_we = R_wb_current*R_eb'; %rotation of ee frame wrt world frame
Jb = Ad_eb\(blkdiag(R_we',R_we')*Je);
Jb_t = Jb';
iJb = pinv(Jb); diJb = zeros(7,6);
Te = LWR.fkine(q');
C_o = blkdiag(skew(zyx2R(o_ref(1,:))'*w_ref(:,1))*M_o(1),skew(zyx2R(o_ref(1,:))'*w_ref(:,1))*M_o(4:6,4:6));
N_o = [zyx2R(o_ref(1,:))'*[0;0;M_o(1)*9.8];0;0;0];
M_t = iJb'*M_m*iJb + M_o;
C_t = iJb'*(C_m*iJb + M_m*diJb) + C_o; % it is approximated later
N_t = iJb'*N_m + N_o;

C_t = C_o*x_sim(7:12,1); %approximation for now

%% plot
if(show_video)
    subplot(3,2,[1,3,5])
    line([pee0(1),pee1(1)], [pee0(2),pee1(2)], [pee0(3),pee1(3)], 'LineStyle', '--', 'LineWidth', 3)
    LWR.plot(q_initial');
    view(-90,45);
    subplot(3,2,2)
    obj_handle = o.plotRB(h_object_fig, eye(4), 1, 1, zeros(6,1), zeros(12,1));
    axis([-0.1 0.1 -0.1 0.1 -0.05 0.05])
    view(65,20);
    
    if(save_video)
        frames = [];
    end
end

for ii=1:n_sim

	% set x0
    fprintf('iter: %d\n', ii);
	ocp.set('constr_x0', x_sim(:,ii));

	% set trajectory initialization
	ocp.set('init_x', x_traj_init);
	ocp.set('init_u', u_traj_init);
    
    % set parameter
    for k=0:ocp_N-1
        ocp.set('p', [M_o(:); C_o(:); N_o(:); Jb_t(:);...
            M_t(:); C_t(:); N_t(:); x_traj_init(7:12,k+1)], k);
    end
    
    for k = 0:ocp_N-1 %new - set the reference to track
        yref = ref(zyx2R(x_sim(4:6,ii)), p_ref, pd_ref, ocp_N, k+ii, o_ref, w_ref,lm_ref(1,:)');
        yref = [yref;zeros(16,1)];
        ocp.set('cost_y_ref', yref, k);
    end
    yref_e = ref(zyx2R(x_sim(4:6,ii)), p_ref, pd_ref, ocp_N, k+ii, o_ref, w_ref,lm_ref(1,:)');
    ocp.set('cost_y_ref_e', yref_e, ocp_N);
    
	% solve OCP
	ocp.solve();
    
    logger.cost(end+1) = ocp.get_cost();
    
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
    sim.set('p', [M_o(:); C_o(:); N_o(:); Jb_t(:);...
            M_t(:); C_t(:); N_t(:); x_traj_init(7:12,k+1)]);

	% simulate state
	sim.solve();
    
	% get new state
	x_sim(:,ii+1) = sim.get('xn');
    
    % update dynamic matrices
    q_prev = q;
    R_wb_current = rotz(x_sim(4,ii+1))*roty(x_sim(5,ii+1))*rotx(x_sim(6,ii+1));
    T_wb_current = [R_wb_current,x_sim(1:3,ii+1);0 0 0 1];
    T_we_current = T_wb_current/T_eb;
    q = LWR.ikcon(T_we_current,q_prev')';
    %     dq_current = (q_current-q_prev)/h;
    Je = LWR.jacob0(q');
    R_we = R_wb_current*R_eb'; %rotation of ee frame wrt world frame
    Jb = Ad_eb\(blkdiag(R_we',R_we')*Je);
    Jb_t = Jb';
    iJb = pinv(Jb); diJb = zeros(7,6);
    Jb_t = Jb';
    dq = pinv(Jb)*x_sim(7:12,ii+1);
%     q_log(:,end+1) = q;
%     dq_log(:,end+1) = dq;
    logger.q(:,end+1) = q;
    logger.dq(:,end+1) = dq;
    
    Te = LWR.fkine(q');
    M_m = LWR.inertia(q');
    C_m = LWR.coriolis(q',dq');
    N_m = LWR.gravload(q')';
    C_o = blkdiag(skew(x_sim(10:12,ii+1))*M_o(1),skew(x_sim(10:12,ii+1))*M_o(4:6,4:6));
    N_o = [zyx2R(x_sim(4:6,ii+1))'*[0;0;M_o(1)*9.8];0;0;0];

    M_t = iJb'*M_m*iJb + M_o;
    C_t = C_o*x_sim(7:12,ii+1); % iJb'*(C_m*iJb + M_m*diJb) + C_o;
    N_t = iJb'*N_m + N_o;
    
    logger.l(:,end+1) = x_sim(14:29,ii+1);
    Fc = o.Fc_hat*logger.l(:,end);
    Fb = (o.G*Fc);
    %Fb = (M_o*(x0(7:12) - xp(7:12))/h + C_o*x0(7:12) + N_o);
    
    %(M_t*(x0(7:12) - xx(7:12,mpciter+1))/h + C_t*x0(7:12) + N_t);
    logger.tau(:,end+1) = Jb'*(M_t*(x_sim(7:12,ii+1) - x_sim(7:12,ii))/dt + C_t + N_t);
    
    ep = yref(1:3) - x_sim(1:3,ii+1);
    eo = yref(4:6) - x_sim(4:6,ii+1);
    logger.e(:,end+1) = [norm(ep), norm(eo)]';
    
    for i = 1:7
        h_torque(i).XData = 1:ii;
        h_torque(i).YData = logger.tau(i, 1:ii);
    end
    for i = 1:2
        h_error(i).XData = 1:ii;
        h_error(i).YData = logger.e(i, 1:ii);
    end
    
    %% plot
    if(show_video)
        subplot(3,2,[1,3,5])
        LWR.plot(q');
        %view(-90,45);
        view(90,0)
        subplot(3,2,2)
        delete(obj_handle);
        obj_handle = o.plotRB(h_object_fig, eye(4), 1, 1, Fb, Fc);
        axis([-0.1 0.1 -0.1 0.1 -0.05 0.05])
        view(65,20);

        drawnow;
        
        if(save_video)
            frames = [frames, getframe(gcf)];
        end
    end
end

main_loop_time = toc(main_loop);
average_mpc_time = main_loop_time/n_sim

status = ocp.get('status');

if status==0
	fprintf('\nsuccess!\n\n');
else
	fprintf('\nsolution failed!\n\n');
end

%%
if(show_video && save_video)
    strExtensionAvi = '.avi';
    strExtensionMp4 = '.mp4';
    strVidAvi = strcat(video_title,strExtensionAvi);
    strVidMp4 = strcat(video_title,strExtensionMp4);
    
    vidObjAvi = VideoWriter(strVidAvi);
    vidObjAvi.FrameRate = 20;
    % Open the video file.
    disp('Opening the video ...')
    open(vidObjAvi);
    disp('Making the video...')
    for k=1:length(frames)
        % Write each frame to the file.
        writeVideo(vidObjAvi,frames(k));
    end
    
    % Close the video file.
    disp('Closing the video...')
    close(vidObjAvi);
    
    % disp('For some reason the following lines do not work if I execute them in MATLAB but they do work if you paste them one by one in your terminal:')
    % Remove the old mp4 video
    % system(sprintf('rm %s',strVidMp4))
    % Compress the video created (from avi to mp4)
    fprintf('$>: ffmpeg -i %s -c:v libx264 -crf 20 -preset veryslow -profile:v baseline -level 3.0 %s\n\n',strVidAvi,strVidMp4)
    % % Remove the avi video created
    % system(sprintf('rm %s',strVidAvi))
    
    % Clear useless variables
    clear strVideo strExtensionMp4 strExtensionAvi time distIJ distIJEst
end


if(show_plots)
    %% MPC outputs velocity in body frame, rotate it to plot in W frame
    for i = 1:size(x_sim,2)
        pd_op(:,i) = zyx2R(x_sim(4:6,i))*x_sim(7:9,i);
        wd_op(:,i) = zyx2R(x_sim(4:6,i))*x_sim(10:12,i);
    end
    
    contactF_final = o.Fc_hat*x_sim(14:end,:);
    bodyF_final = o.G*contactF_final;
end

%% function definition
function y = ref(R,p,pd,k,instant,ang,wd,lambda)
    if(instant<=size(p,1)-k)
        y = [];
        y = [y; [p(instant,:)'; ang(instant,:)']; [R'*pd(instant,:)'; R'*wd(:,instant)];9.8;lambda];
    else
        y = [];
        y = [y; [p(size(p,1),:)'; ang(size(p,1),:)']; [R'*pd(size(p,1),:)'; R'*wd(:,size(p,1))];9.8;lambda];
    end
end