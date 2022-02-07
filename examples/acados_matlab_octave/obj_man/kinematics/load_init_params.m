%% contact params
mu = 0.5;
h1 = trvec2tform([-0.02,  0.02, -0.02]); % position of {C_i} in {B}
h2 = trvec2tform([ 0.02,  0.02, -0.02]);
h3 = trvec2tform([ 0.02, -0.02, -0.02]);
h4 = trvec2tform([-0.02, -0.02, -0.02]);
c4 = contact(h4, 'pcwf', mu);
c3 = contact(h3, 'pcwf', mu);
c2 = contact(h2, 'pcwf', mu);
c1 = contact(h1, 'pcwf', mu);

%% object params
m = 0.5;
I = 1e-4*eye(3);

%% create rigid body
o = rigidBody(m, I);
o.addContact([c1,c2,c3,c4]);
clear h1 h2 h3 h4 c1 c2 c3 c4;
o.reset()

%% trajectory params
%plan_trajectory
%plan trajectory in spatial frame 
%planned traj is for object and not the EE (use inverse kin to get EE traj)
dt = 0.01;
T = params.sim_tim;
tSteps = 0:dt:T; % time vector in seconds
[s, sd, sdd] = tpoly(0, 1, tSteps'); % curvilinear abscissa and time derivatives
tz_eb = 0.03; %0.02 + 0.016; 
ty_eb = 0; % 0.08; 
global R_eb
R_eb = eye(3); % rotation of body frame wrt ee
T_eb = [R_eb,[0;ty_eb;tz_eb];0 0 0 1];

%% linear trajectory (position)
pee0 = [0.417,-0.417,0.424]'; %for q = [-45,-30,0,60,0,90] %initial pos of ee wrt world
pee1 = [0.417,0.417,0.424]'; %for q = [45,-30,0,60,0,90] %final pos of ee wrt world
oee0 = [-pi/4,0,0]; %initial euler zyx of ee wrt world
oee1 = [-pi/4,0,0]; % euler zyx final

% pee0 = [-0.541,0.363,0.078]'; %for q = [-45,-30,0,60,0,90] %initial pos of ee wrt world
% pee1 = [-0.541,-0.363,0.078]'; %for q = [45,-30,0,60,0,90] %final pos of ee wrt world
% oee0 = [-1.2,0,0]; %initial euler zyx of ee wrt world
% oee1 = [-1.2,0,0]; % euler zyx final
% % oee1 = deg2rad([33.7,25.7,-16.1] + [0,-20,10]); %final euler zyx of ee wrt world

T0_we = [rotz(oee0(1))*roty(oee0(2))*rotx(oee0(3)), pee0; 0 0 0 1]; %initial transf of ee
q_initial = LWR.ikcon(T0_we)';
% q_initial = [0;1.57;-1.57;-1.2;1.57;-1.57;0];
T0_wb = T0_we*T_eb; %initial transf of body
p0 = T0_wb(1:3,4); %initial pos of body wrt world

T1_we = [rotz(oee1(1))*roty(oee1(2))*rotx(oee1(3)),pee1;0 0 0 1]; %final transf of ee
% q_final = LWR.ikcon(T1_we)';
T1_wb = T1_we*T_eb;
p1 = T1_wb(1:3,4); 
od(1,1:3) = [0 0 0]; w_des(1:3,1) = [0 0 0];

for i = 1 : size(tSteps,2)
    p(i, 1:3) = (p0 + s(i)*(p1-p0))';
    pd(i, 1:3) = (sd(i)*(p1-p0))';
    pdd(i, 1:3) = (sdd(i)*(p1-p0))';
    
    o_wb(i,1:3) = (oee0 + s(i)*(oee1-oee0))';
    od(i,1:3) = sd(i)*(oee1-oee0);
    w_des(1:3,i) = zyx2E(o_wb(i,:))*od(i,:)';
end
%% circular trajectory 
%q_init = [13.0814;-69.7241;2.6837;28.0247;0.0670;-4.1461];

% rho = 0.2; % radius
% c = [-0.1, 0, 0.814]'; % center
% R = roty(pi/2); % rotation
% 
% pc = [rho*sin(2*pi*s), rho*cos(2*pi*s), zeros(size(s,1),1)];
% pdc = [rho*2*pi*cos(2*pi*s).*sd, -rho*2*pi*sin(2*pi*s).*sd, zeros(size(s,1),1)];
% pddc = [-rho*4*pi^2*sin(2*pi*s).*sd + rho*2*pi*cos(2*pi*s).*sdd, ...
%          rho*4*pi^2*cos(2*pi*s).*sd - rho*2*pi*sin(2*pi*s).*sdd, ... 
%          zeros(size(s,1),1)];
%      
% for i = 1 : size(tSteps,2)
%     pc(i, 1:3) = (R*(pc(i, 1:3)' + c))';
%     pdc(i, 1:3) = (R*pdc(i, 1:3)')';
%     pddc(i, 1:3) = (R*pddc(i, 1:3)')'; 
% end     
% 
% pcv(1, 1:3) = pc(1, 1:3);
% for i = 2 : size(tSteps,2)
%     pcv(i, 1:3) = pcv(i-1,1:3) + pdc(i, 1:3)*dt;
% end 
%% circular trajectory given a staring point in KUKA_petercorke
% % pee_first = [0.8140;0.2;0.04]; %manipulator EE starting point
% pee_first = [0.593;0.342;0.083]; %manipulator EE starting point for q=(30;-60;0;60;0;120;0)
% oee0 = [pi/6,0,0]; % euler zyx initial
% p_first = pee_first + [0;0;tz_eb]';
% rho = 0.2; % radius
% c = [p_first(1),p_first(2)-rho,p_first(3)]';
% pc = [zeros(size(s,1),1), rho*cos(2*pi*s), rho*sin(2*pi*s)] + repmat(c',size(s,1),1);
% pdc = [zeros(size(s,1),1), -rho*2*pi*sin(2*pi*s).*sd, rho*2*pi*cos(2*pi*s).*sd];
% pddc = [zeros(size(s,1),1),-rho*4*pi^2*cos(2*pi*s).*sd - rho*2*pi*sin(2*pi*s).*sdd, ...
%          -rho*4*pi^2*sin(2*pi*s).*sd + rho*2*pi*cos(2*pi*s).*sdd];
% o_wb = oee0.*[cos(2*pi*s),zeros(size(s)),zeros(size(s))];%repmat(oee0,size(pc,1),1);
% od = oee0.*[-2*pi*sin(2*pi*s).*sd,zeros(size(s)),zeros(size(s))];%zeros(flip(size(pc)));
% for i=1:size(o_wb,1)
%     w_des(1:3,i) = zyxE(o_wb(i,:))*od(i,:)';
% end
%% sinusoidal trajectory given a staring point in KUKA_petercorke
% pee_first = [0.593;0.342;0.083]; %manipulator EE starting point for 
% q_first = deg2rad([-45,-30,0,60,0,90,0])';
% oee0 = [pi/6,0,0]; % euler zyx initial
% oee1 = [pi/6,0,0]; % euler zyx final
% oee_inter = deg2rad([33.7,25.7,-16.1]); % euler zyx initial
% p_first = pee_first + [0;0;tz_eb];
% rho = 0.2; % amplitude
% pc = [zeros(size(s,1),1), -0.75*s, rho*sin(pi*s)] + repmat(p_first',size(s,1),1);
% pdc = [zeros(size(s,1),1), -0.75*sd, rho*pi*cos(pi*s).*sd];
% pddc = [zeros(size(s,1),1), -0.75*sdd, ...
%          -rho*pi^2*sin(pi*s).*sd + rho*pi*cos(pi*s).*sdd];
% % o_wb = (oee_inter-oee0).*[sin(pi*s),sin(pi*s),sin(pi*s)] + repmat(oee0,size(s,1),1);%repmat(oee0,size(pc,1),1);
% % od = (oee_inter-oee0).*[pi*cos(pi*s).*sd,pi*cos(pi*s).*sd,pi*cos(pi*s).*sd];%zeros(flip(size(pc)));
% % for i=1:size(o_wb,1)
% %     w_des(1:3,i) = zyxE(o_wb(i,:))*od(i,:)';
% % end
% for i=1:size(tSteps,2)
%     o_wb(i,1:3) = (oee0 + s(i)*(oee_inter-oee0));
%     od(i,1:3) = sd(i)*(oee_inter-oee0);
%     w_des(1:3,i) = zyxE(o_wb(i,:))*od(i,:)';
% end
%% bang-bang profile
% dt = 0.01;
% T = 1;
% tSteps = 0:dt:T; % time vector in seconds
% pdd0 = [1,1,1];
% pdd1 = -pdd0;
% pd(1,1:3) = [0,0,0];
% p(1,1:3) = [0,0,0];
% pdd(1,1:3) = [0,0,0];
% for i=2:size(tSteps,2)
%     if ((i*dt)<=0.5)
%         pd(i,1:3) = pd(i-1,1:3) + pdd0*dt;
%         p(i,1:3) = p(i-1,1:3) + pd(i,1:3)*dt + 0.5*pdd0*dt^2;
%         pdd(i,1:3) = pdd0;
%     else
%         pd(i,1:3) = pd(i-1,1:3) + pdd1*dt;
%         p(i,1:3) = p(i-1,1:3) + pd(i,1:3)*dt + 0.5*pdd1*dt^2;
%         pdd(i,1:3) = pdd1;
%     end
% end

%% spiral profile
% rho = 0.15; % radius
% c = [0, -rho, 0]'; % center
% R = rotx(pi/2); % rotation
% ft = 5; %and keep T=10
% pc = [rho*sin(2*pi*tSteps'/ft), rho*cos(2*pi*tSteps'/ft),tSteps'/ft];
% pdc = [rho*2*pi*cos(2*pi*tSteps'/ft)/ft, -rho*2*pi*sin(2*pi*tSteps'/ft)/ft, ones(size(tSteps',1),1)/ft];
% pddc = [-rho*4*pi^2*sin(2*pi*tSteps'/ft)/(ft^2), ...
%          -rho*4*pi^2*cos(2*pi*tSteps'/ft)/(ft^2), ... 
%          zeros(size(tSteps',1),1)];   

%% set reference values
p_ref = p;
pd_ref = pd;
pdd_ref = pdd;
o_ref = o_wb; 
w_ref = w_des;
