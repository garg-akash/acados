clear
clc
close all

load_rodyman_dynamic_params;

USE_COPPELIA = 1;

if(USE_COPPELIA)
    coppelia = remApi('remoteApi');
    coppelia.simxFinish(-1);
    clientID = coppelia.simxStart('127.0.0.1',19997,true,true,5000,5);
    if (clientID>-1)
        disp('remote API server connected successfully!');
        handle_joint = zeros(9,1);
        [rtn, handle_joint(1)] = coppelia.simxGetObjectHandle( clientID, 'Joint_torso_1_ID17',  coppelia.simx_opmode_oneshot_wait);
        [rtn, handle_joint(2)] = coppelia.simxGetObjectHandle( clientID, 'Joint_torso_2_ID18',  coppelia.simx_opmode_oneshot_wait);
        [rtn, handle_joint(3)] = coppelia.simxGetObjectHandle( clientID, 'Joint_arm_dx_1_ID19', coppelia.simx_opmode_oneshot_wait);
        [rtn, handle_joint(4)] = coppelia.simxGetObjectHandle( clientID, 'Joint_arm_dx_2_ID5',  coppelia.simx_opmode_oneshot_wait);
        [rtn, handle_joint(5)] = coppelia.simxGetObjectHandle( clientID, 'Joint_arm_dx_3_ID6',  coppelia.simx_opmode_oneshot_wait);
        [rtn, handle_joint(6)] = coppelia.simxGetObjectHandle( clientID, 'Joint_arm_dx_4_ID7',  coppelia.simx_opmode_oneshot_wait);
        [rtn, handle_joint(7)] = coppelia.simxGetObjectHandle( clientID, 'Joint_arm_dx_5_ID8',  coppelia.simx_opmode_oneshot_wait);
        [rtn, handle_joint(8)] = coppelia.simxGetObjectHandle( clientID, 'Joint_arm_dx_6_ID9',  coppelia.simx_opmode_oneshot_wait);
        [rtn, handle_joint(9)] = coppelia.simxGetObjectHandle( clientID, 'Joint_arm_dx_7_ID10', coppelia.simx_opmode_oneshot_wait);
        disp('joint handles retrieved!');
        [rtn, handle_object] = coppelia.simxGetObjectHandle( clientID, 'Cuboid', coppelia.simx_opmode_oneshot_wait);
        disp('object handle retrieved!');
    else
        disp('failed connecting to remote API server!');
        coppelia.simxFinish(clientID);  % close the line if still open
        coppelia.delete();              % call the destructor!
        return
    end
end

%simxSetJointPosition(clientID, number jointHandle, number position, number operationMode);

%% Robot
% define params
disp('Creating the robot...')
global Rodyman
a = zeros(10,1); a(2)=0.1655; a(6)=0.35;
alpha = zeros(10,1); alpha(1) = pi/2; alpha(3) = 5*pi/6; alpha(4) = -pi/2;
alpha(5) = -pi/2; alpha(7) = pi/2; alpha(8) = -pi/2; alpha(9) = pi/2;
d = zeros(10,1); d(1)=1.2277; d(4)=0.298; d(8)=0.305; d(10)=0.075;
theta = zeros(10,1);
offset = zeros(10,1); offset(1) = pi; offset(2) = pi/2; offset(3) = pi/2;
offset(4) = pi/2; offset(5) = -7*pi/6; offset(7) = pi/2;

jnt_nr = 9;

% construct links from DH params: theta    d      a     alpha
clear L
for i=1:jnt_nr+1
    m = eval(strcat('m',int2str(i)));
    Ixx = eval(strcat('Ixx',int2str(i)));
    Iyy = eval(strcat('Iyy',int2str(i)));
    Izz = eval(strcat('Izz',int2str(i)));
    rx = eval(strcat('mpx',int2str(i)))/m;
    ry = eval(strcat('mpy',int2str(i)))/m;
    rz = eval(strcat('mpz',int2str(i)))/m;
    L(i) = Revolute('a', a(i), 'alpha', alpha(i), 'd', d(i), 'offset', offset(i), ...
        'I', [Ixx Iyy Izz], 'r', [rx ry rz], 'm', m);
end

% construct robot
Rodyman = SerialLink(L, 'name', 'Rodyman');

% set limits
Rodyman.qlim(1,1) = -170*pi/180;
Rodyman.qlim(1,2) = 170*pi/180;
Rodyman.qlim(2,1) = -120*pi/180;
Rodyman.qlim(2,2) = 120*pi/180;
Rodyman.qlim(3,1) = 0; % fixed
Rodyman.qlim(3,2) = 0;
Rodyman.qlim(4,1) = -120*pi/180;
Rodyman.qlim(4,2) = 120*pi/180;
Rodyman.qlim(5,1) = -170*pi/180;
Rodyman.qlim(5,2) = 170*pi/180;
Rodyman.qlim(6,1) = -120*pi/180;
Rodyman.qlim(6,2) = 120*pi/180;
Rodyman.qlim(7,1) = -170*pi/180;
Rodyman.qlim(7,2) = 170*pi/180;
Rodyman.qlim(8,1) = -170*pi/180;
Rodyman.qlim(8,2) = 170*pi/180;
Rodyman.qlim(9,1) = -170*pi/180;
Rodyman.qlim(9,2) = 170*pi/180;
Rodyman.qlim(10,1) = -170*pi/180;
Rodyman.qlim(10,2) = 170*pi/180;

% set initial configuration
if(~USE_COPPELIA)
    Rodyman.plot(zeros(10,1)', 'scale', 0.5)
end
Td = [[0 -1, 0; 1, 0, 0; 0, 0, 1], [0.7, -0.5, 1.34]'; 0 0 0 1];
q_0 = [0.4177, 0.4658, 0, -1.2741, -0.4330, 2.0929, 0.6914, 0.4386, 1.6432, -1.1352]'; %% put init values
q_0 = Rodyman.ikcon(Td,q_0);
T0_we = Rodyman.fkine(q_0);

if(~USE_COPPELIA)
    Rodyman.plot(q_0);
end

%% Object
mu = 0.5;%0.75;
obj_len = 0.04;
obj_wd = 0.04;
obj_ht = 0.04;
h1 = trvec2tform([-obj_len/2, obj_wd/2, -obj_ht/2]); % position of {C_i} in {B}
h2 = trvec2tform([ obj_len/2, obj_wd/2, -obj_ht/2]);
h3 = trvec2tform([ obj_len/2, -obj_wd/2, -obj_ht/2]);
h4 = trvec2tform([-obj_len/2, -obj_wd/2, -obj_ht/2]);
c4 = contact(h4, 'pcwf', mu);
c3 = contact(h3, 'pcwf', mu);
c2 = contact(h2, 'pcwf', mu);
c1 = contact(h1, 'pcwf', mu);

% object params
m = 0.5; % for cube in lab 0.236;
I = 1e-4*eye(3); %for cube in lab 4.5375*1e-5*eye(3);

% create rigid body
o = rigidBody(m, I);
o.addContact([c1,c2,c3,c4]);
clear h1 h2 h3 h4 c1 c2 c3 c4;
o.reset()
Fc_hat = blkdiag(o.contacts(1).x_hat_, o.contacts(2).x_hat_, o.contacts(3).x_hat_, o.contacts(4).x_hat_);

%% Trajectory
disp('Planning the trajectory...')
%plan_trajectory
dt = 0.005; % 0.01;
T = 1.5;
tSteps = 0:dt:T; % time vector in seconds
[s, sd, sdd] = tpoly(0, 1, tSteps'); % curvilinear abscissa and time derivatives
ty_eb = 0.075;
tz_eb = 0.07 + 0.0275;
R_eb = eye(3); % rotation of body frame wrt ee
T_eb = [R_eb,[0;ty_eb;tz_eb];0 0 0 1];
Ad_eb = [[R_eb,skew(T_eb(1:3,4))*R_eb];[zeros(3),R_eb]];

% ee linear trajectory (position)
pee0 = T0_we.t;
pee1 = T0_we.t+[0,0.65,0]';
eul = rotm2eul(T0_we.R,'ZYX')';
oee0 = eul; %initial euler zyx of ee wrt world
oee1 = eul; % euler zyx final

T0_we = [rotz(oee0(1))*roty(oee0(2))*rotx(oee0(3)), pee0; 0 0 0 1]; % initial transf of ee wrt world
T0_wb = T0_we*T_eb; % initial transf of body wrt world
p0 = T0_wb(1:3,4); % initial pos of body wrt world

T1_we = [rotz(oee1(1))*roty(oee1(2))*rotx(oee1(3)), pee1;0 0 0 1]; %final transf of ee
T1_wb = T1_we*T_eb;
p1 = T1_wb(1:3,4);

od(1,1:3) = [0 0 0]; w_des(1:3,1) = [0 0 0];

for i = 1 : size(tSteps,2)
    p_ref(i, 1:3) = (p0 + s(i)*(p1-p0))';
    pd_ref(i, 1:3) = (sd(i)*(p1-p0))';
    pdd_ref(i, 1:3) = (sdd(i)*(p1-p0))';
    
    o_ref(i,1:3) = (oee0 + s(i)*(oee1-oee0))';
    od(i,1:3) = sd(i)*(oee1-oee0);
    w_ref(1:3,i) = zyx2E(o_ref(i,:))*od(i,:)';
end

disp('Calculating inverse kinematics...')
q_ref = [];
dq_ref = [];
q_new = q_0;  %% put init values
for i = 1:size(p_ref,1)
    R_wb = rotz(o_ref(i,1))*roty(o_ref(i,2))*rotx(o_ref(i,3));
    T_wb = [R_wb,p_ref(i,:)';0 0 0 1];
    T_we = T_wb/T_eb;
    J = Rodyman.jacob0(q_new);
    J(:,3) = [];
    dq_new = pinv(J)*[pd_ref(i,:)';w_ref(:,i)];
    q_new = Rodyman.ikcon(T_we, q_new);
    
    q_ref = [q_ref [q_new(1:2), q_new(4:end)]'];
    dq_ref = [dq_ref dq_new];
end

fileID = fopen('q_ref.txt','w');
fprintf(fileID,'%12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n', q_ref);
fclose(fileID);


%% MPC
%%
q_log = []; dq_log = [];
tau_log = []; dtau_log = [];
p_log = []; phi_log = [];
p_ref_log = []; phi_ref_log = [];
Te = Rodyman.fkine(toRodyman(q_ref(:,1))');
Je_current = Rodyman.jacob0(toRodyman(q_ref(:,1))');
Je_current(:,3) = [];
T_wb = Te.T*T_eb;

M_m = Rodyman.inertia(toRodyman(q_ref(:,1))'); M_m(3,:) = []; M_m(:,3) = [];
M_inv = inv(M_m);
C_m = Rodyman.coriolis(toRodyman(q_ref(:,1))', toRodyman(dq_ref(:,1))');
C_m(3,:) = []; C_m(:,3) = [];
C_m = C_m*dq_ref(:,1);
N_m = Rodyman.gravload(toRodyman(q_ref(:,1))')'; %outputs col vector...take transpose
N_m(3,:) = [];

Jb = Ad_eb\(blkdiag(Te.R',Te.R')*Je_current);
Jb_dot = zeros(6,9);

o.computeN(Te.R);
C_o = blkdiag(skew(zyx2R(o_ref(1,:))'*w_ref(:,1))*o.Mb(1),skew(zyx2R(o_ref(1,:))'*w_ref(:,1))*o.Mb(4:6,4:6));

M_tilde = M_m + Jb'*o.Mb*Jb;
M_tilde_inv = inv(M_tilde);
% C_tilde = C_m + Jb'*o.Mb*Jb_dot + Jb'*C_o*Jb;
N_tilde = N_m + Jb'*o.Nb;

disp('Generating the MPC controller...')
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
%ocp_nlp_solver = 'sqp_rti';
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
ocp_levenberg_marquardt = 1e-2;

%% setup problem
LAMBDA_CONST = 1;
% manipulator mpc
model = rodyman_object_model_lambda(o.G);
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
Q = blkdiag(1e-2*eye(9), 1e7*eye(9), 1e5*eye(9), 1e0*eye(16));
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

x0 = [N_tilde; q_ref(:,1); zeros(9,1); pinv(o.G*Fc_hat)*o.Nb];
Jbx = zeros(nbx, nx); for ii=1:nbx Jbx(ii,ii)=1.0; end
lbx = [-176; -176; -110; -110; -110; -40; -40; -40; -40;... % tau
    deg2rad(-170);deg2rad(-120);deg2rad(-170); ...
    deg2rad(-120);deg2rad(-170);deg2rad(-120); ...
    deg2rad(-170);deg2rad(-120);deg2rad(-170); ... % q
    deg2rad(-98);deg2rad(-98);deg2rad(-100); ...
    deg2rad(-98);deg2rad(-140);deg2rad(-180); ...
    deg2rad(-180);deg2rad(-140);deg2rad(-180); ... % qdot
    1e-3*ones(16,1)];   % lambda
ubx = -lbx;
ubx(28:27+16) = 2;

Jbu = zeros(nbu, nu); for ii=1:nbu Jbu(ii,ii)=1.0; end
lbu = -1000*ones(nu, 1);
ubu = 1000*ones(nu, 1);

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
%ocp_opts.set('qp_solver_warm_start', 0);

ocp_opts.opts_struct

%% acados ocp
% create ocp
ocp = acados_ocp(ocp_model, ocp_opts);
ocp
%ocp.generate_c_code

disp('Simulating...')
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

n_sim = floor(1.5*T/dt);
x_sim = zeros(nx, n_sim+1);
x_sim(:,1) = x0;
u_sim = zeros(nu, n_sim);

% x_traj_init = zeros(nx, ocp_N+1);
x_traj_init = repmat(x0, 1, ocp_N+1);
u_traj_init = zeros(nu, ocp_N);

% lambda_log = pinv(o.G*Fc_hat)*(o.Mb*(Jb*(x_sim(19:27,1)-zeros(7,1))/dt) + ...
%                 C_o*Jb*x_sim(19:27,ii+1) + o.Nb);
% lambda_log = pinv(o.G*Fc_hat)*(o.Mb*(Jb*(x_sim(19:27,1)-zeros(7,1))/dt) + ...
%                  o.Nb);

Fb_read = o.Nb;
Fc_read = pinv(o.G)*Fb_read;
ddq = zeros(9,1);
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
    x0(28:43) = La_read;
    ocp.set('constr_x0', x0);
    
    % set parameter
    for k=0:ocp_N-1
        %         ocp.set('p', [M_tilde_inv(:); C_m(:); N_tilde(:); Jb(:); o.Mb(:); C_o(:); o.Nb(:); x_sim(19:27,ii)], k);
        ocp.set('p', [M_tilde(:); C_m(:); N_tilde(:); Jb(:); ...
            o.Mb(:); C_o(:); o.Nb(:); zeros(54,1); ...
            M_m(:); C_m(:); N_m(:); Fb_read; mu], k);
    end
    
    % compute reference
    for k = 0:ocp_N-1 %new - set the reference to track
        yref = [ref(ocp_N, k+ii, q_ref, dq_ref); lambda_ref; zeros(9,1);];
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
        o.Mb(:); C_o(:); o.Nb(:); zeros(54,1); ...
        M_m(:); C_m(:); N_m(:); Fb_read; mu]);
    
    % solve
    sim_status = sim.solve();
    if sim_status ~= 0
        disp(['acados integrator returned error status ', num2str(sim_status)])
    end
    
    % get new state
    x_sim(:,ii+1) = sim.get('xn');
    u_sim(:,ii) = u0;
    
    %if (ii >= 100 && ii <= 150) % disturbance torque
    %    x_sim(1:7,ii+1) = x_sim(1:7,ii+1) + Jb'*[20*sin((ii - 100)/50*pi);0; 0; 0; 0; 0];
    %end
    
    % update dynamic matrices
    
    Te = Rodyman.fkine(toRodyman(x_sim(10:18,ii+1))');
    T_wb = Te.T*T_eb;
    Je_current = Rodyman.jacob0(toRodyman(x_sim(10:18,ii+1))');
    Je_current(:,3) = [];
    
    
    M_m = Rodyman.inertia(toRodyman(x_sim(10:18,ii+1))'); M_m(3,:) = []; M_m(:,3) = [];
    M_inv = inv(M_m);
    C_m = Rodyman.coriolis(toRodyman(x_sim(10:18,ii+1))', toRodyman(x_sim(19:27,ii+1))');
    C_m(3,:) = []; C_m(:,3) = [];
    C_m = C_m*x_sim(19:27,ii+1);
    N_m = Rodyman.gravload(toRodyman(x_sim(10:18,ii+1))')'; %outputs col vector...take transpose
    N_m(3,:) = [];
    
    Jb = Ad_eb\(blkdiag(Te.R',Te.R')*Je_current);
    
    o.computeN(Te.R);
    xdot = Jb*x_sim(19:27,ii+1);
    C_o = blkdiag(skew(xdot(4:6))*o.Mb(1),skew(xdot(4:6))*o.Mb(4:6,4:6));
    
    M_tilde = M_m + Jb'*o.Mb*Jb;
    M_tilde_inv = inv(M_tilde);
    C_tilde = C_m + Jb'*o.Mb*Jb_dot + Jb'*C_o*Jb;
    N_tilde = N_m + Jb'*o.Nb;
    
    %     lambda_log = [lambda_log pinv(o.G*Fc_hat)*(o.Mb*(Jb*(x_sim(19:27,ii+1)-...
    %         x_sim(19:27,ii))/dt) + o.Nb)];
    
    %ddq = pinv(M_m)*(x_sim(1:7,ii) - Jb'*Fb_read - C_m - N_m);
    ddq = M_tilde_inv*(x_sim(1:9,ii+1) - C_tilde*x_sim(19:27,ii+1) - N_tilde);
    ddx = Jb*ddq + Jb_dot*x_sim(19:27,ii+1);
    Fb_read = o.Mb*ddx + o.Nb;
    Fc_read = pinv(o.G)*Fb_read;
    La_read = pinv(Fc_hat)*Fc_read;
    
    % store logs
    p_log = [p_log, T_wb(1:3,4)];
    phi_log = [phi_log, rpy(T_wb(1:3,1:3))];
    lambda_log = [lambda_log La_read];
    dq_log = [dq_log, x_sim(19:27,ii+1)];
    q_log = [q_log, x_sim(10:18,ii+1)];
    tau_log = [tau_log, x_sim(1:9,ii+1)];
    dtau_log = [dtau_log, u_sim(1:9,ii)];
    
    status = ocp.get('status');
    
    if status==0
        fprintf('\nsuccess!\n\n');
    else
        fprintf('\nsolution failed with status %d!\n\n', status);
        break;
    end
    
end

avg_time_solve = toc/n_sim

if(USE_COPPELIA)
    q = [q_0(1:2), q_0(4:end)];
    q = toSim(q);
    for i=1:size(handle_joint)
        [err]=coppelia.simxSetJointPosition(clientID,handle_joint(i),q(i),coppelia.simx_opmode_oneshot_wait);
        if (err~=coppelia.simx_return_ok)
            fprintf('failed to set the position of joint %d \n',i);
        end
    end
    coppelia.simxSetObjectPosition(clientID,handle_object,-1,[0.7,-0.42,1.4376],coppelia.simx_opmode_oneshot_wait)
    coppelia.simxSetObjectOrientation(clientID,handle_object,-1,[0,0,0],coppelia.simx_opmode_oneshot_wait)
end

if(USE_COPPELIA)
    for ii = 1:size(x_sim,2)-1
        q = toSim(x_sim(10:18,ii+1));
        for i=1:size(handle_joint)
            [err]=coppelia.simxSetJointPosition(clientID,handle_joint(i),q(i),coppelia.simx_opmode_oneshot);
            if (err~=coppelia.simx_return_ok)
                fprintf('failed to set the position of joint %d \n',i);
            end
        end
        pause(0.005);
    end
end

if(USE_COPPELIA)
    q = toSim(x_sim(10:18,1));
    for i=1:size(handle_joint)
        [err]=coppelia.simxSetJointPosition(clientID,handle_joint(i),q(i),coppelia.simx_opmode_oneshot);
        if (err~=coppelia.simx_return_ok)
            fprintf('failed to set the position of joint %d \n',i);
        end
    end
    coppelia.simxSetObjectPosition(clientID,handle_object,-1,[0.7,-0.42,1.4376],coppelia.simx_opmode_oneshot_wait)
    coppelia.simxSetObjectOrientation(clientID,handle_object,-1,[0,0,0],coppelia.simx_opmode_oneshot_wait)
end

%% Write data to a file
fileID = fopen('q_log.txt','w');
fprintf(fileID,'%12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n', q_log);
fclose(fileID);

%% PLOTS
% p
figure
for i = 1:3
    subplot(3,1,i)
    plot(p_log(i,:),'-','linewidth',2)
    grid on
    hold on
    plot(p_ref(:,i)','--','linewidth',2)
    yl = strcat('$p_{', strcat(int2str(i), '}$'));
    xlabel('iteration','Interpreter','latex');ylabel(yl,'Interpreter','latex')
    set(gca, 'FontSize', 10)
end

% phi
figure
for i = 1:3
    subplot(3,1,i)
    plot(phi_log(4-i,:),'-','linewidth',2)
    grid on
    hold on
    plot(o_ref(:,i)','--','linewidth',2)
    yl = strcat('$\phi_{', strcat(int2str(i), '}$'));
    xlabel('iteration','Interpreter','latex');ylabel(yl,'Interpreter','latex')
    set(gca, 'FontSize', 10)
end

% dq
figure
for i = 1:9
    subplot(3,3,i)
    plot(dq_log(i,:),'-','linewidth',2)
    grid on
    hold on
    plot(dq_ref(i,1:size(dq_ref,2)),'--','linewidth',2)
    plot(repmat(lbx(14+i),size(dq_log,2)),'--r','linewidth',2)
    plot(repmat(ubx(14+i),size(dq_log,2)),'--r','linewidth',2)
    yl = strcat('$\dot{q}_', strcat(int2str(i), '$'));
    xlabel('iteration','Interpreter','latex');ylabel(yl,'Interpreter','latex')
    set(gca, 'FontSize', 10)
end

% q
figure
for i = 1:9
    subplot(3,3,i)
    plot(q_log(i,:),'-','linewidth',2)
    grid on
    hold on
    plot(q_ref(i,1:size(q_ref,2)),'--','linewidth',2)
    plot(repmat(lbx(7+i),size(q_log,2)),'--r','linewidth',2)
    plot(repmat(ubx(7+i),size(q_log,2)),'--r','linewidth',2)
    yl = strcat('$q_', strcat(int2str(i), '$'));
    xlabel('iteration','Interpreter','latex');ylabel(yl,'Interpreter','latex')
    set(gca, 'FontSize', 10)
end

% tau
figure
for i = 1:9
    subplot(3,3,i)
    plot(tau_log(i,:),'-','linewidth',2)
    grid on
    hold on
    plot(repmat(lbx(i),size(tau_log,2)),'--r','linewidth',2)
    plot(repmat(ubx(i),size(tau_log,2)),'--r','linewidth',2)
    yl = strcat('$\tau_', strcat(int2str(i), '$'));
    xlabel('iteration','Interpreter','latex');ylabel(yl,'Interpreter','latex')
    set(gca, 'FontSize', 10)
end

% dtau
figure
for i = 1:9
    subplot(3,3,i)
    plot(dtau_log(i,:),'-','linewidth',2)
    grid on
    hold on
    plot(repmat(lbu(i),size(dtau_log,2)),'--r','linewidth',2)
    plot(repmat(ubu(i),size(dtau_log,2)),'--r','linewidth',2)
    yl = strcat('$\dot{\tau}_', strcat(int2str(i), '$'));
    xlabel('iteration','Interpreter','latex');ylabel(yl,'Interpreter','latex')
    set(gca, 'FontSize', 10)
end

% lambda
figure
for i = 1:16
    subplot(4,4,i)
    plot(lambda_log(i,:),'-','linewidth',2)
    grid on
    hold on
    plot(repmat(lbx(27+i),size(lambda_log,2)),'--r','linewidth',2)
    plot(repmat(ubx(27+i),size(lambda_log,2)),'--r','linewidth',2)
    yl = strcat('$\lambda_{', strcat(int2str(i), '}$'));
    xlabel('iteration','Interpreter','latex');ylabel(yl,'Interpreter','latex')
    set(gca, 'FontSize', 10)
end

if(~USE_COPPELIA)
    figure(1)
    Rodyman.plot(toRodyman(q_log)')
end
save('data_rodyman.mat', 'p_log', 'phi_log', 'lambda_log', 'dq_log', 'q_log', 'tau_log', 'dtau_log', 'cost_val_ocp', 'q_ref', 'dq_ref', 'p_ref', 'o_ref', 'lbx', 'ubx', 'lbu', 'ubu')


%% function definition
function y = ref(k,instant,q_init,dq_init)
global Rodyman
if(instant<=size(q_init,2))
    y = [];
    g = Rodyman.gravload(toRodyman(q_init(:,instant))');
    y = [y; [g(1:2), g(4:end)]'; q_init(:,instant); dq_init(:,instant)];%zeros(16,1)];
else
    y = [];
    g = Rodyman.gravload(toRodyman(q_init(:,end))');
    y = [y; [g(1:2), g(4:end)]'; q_init(:,end); dq_init(:,end)];
end
end