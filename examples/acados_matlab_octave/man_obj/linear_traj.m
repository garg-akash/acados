%%%%%
%% TRAJECTORY
dt = 0.008;
T = 1.5; 
x_d = 0;
y_d = 0.62;
z_d = 0;

%%%%%%%%%%%%%%%%%%%
% plan_trajectory %
%%%%%%%%%%%%%%%%%%%

tSteps = 0:dt:T; % time vector in seconds
[s, sd, sdd] = tpoly(0, 1, tSteps'); % curvilinear abscissa and time derivatives

% ee linear trajectory (position)
pee0 = T0_we.t;
pee1 = T0_we.t+[x_d, y_d, z_d]';

eul = rotm2eul(T0_we.R,'ZYX')';
oee0 = eul; %initial euler zyx of ee wrt world
oee1 = eul; % euler zyx final

T0_we = [T0_we.R, T0_we.t; 0 0 0 1]; % initial transf of ee wrt world
T0_wb = T0_we*T_eb; % initial transf of body wrt world
p0 = T0_wb(1:3,4); % initial pos of body wrt world

T1_we = [T0_we(1:3,1:3), pee1;0 0 0 1]; %final transf of ee
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

%%%%%%%%%%%%%%%
%save('reference_linear_traj.mat', 'o_ref', 'od', 'p_ref', 'pd_ref', 'pdd_ref', 'w_ref')