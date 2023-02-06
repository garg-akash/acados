
pee0 = T0_we.t; 
eul = rotm2eul(T0_we.R,'ZYX')';
oee0 = eul; %initial euler zyx of ee wrt world

%%%%%
%% TRAJECTORY
dt = 0.008;
T = 4.5;
time = linspace(0, T, T/dt);

k = 4;                                                              % spline order
t = [0 0.2 0.4 0.6 0.8 1 1.2 1.4 1.6 1.8 2 2.2 2.4 2.6 2.8 3 3.2];  % knot sequence
P = 0.35*[0 0.0 0.0 0.2 0.4  0.2 0 -0.2 -0.4 -0.2 0 0 0;
    0 0.0 0.0 0.2 0.0 -0.2 0 0.2 0 -0.2 0 0 0];                     % control points

traj = trajectory(k, t, P, size(time,2));                           % plan b-spline trajectory
P = [traj.p; zeros(1, size(traj.p,2))];
P = roty(1.57)*rotz(1.57)*P;
V = [diff(P, 1, 2)/dt, [0,0,0]'];                                   % trajectory velocity
A = [diff(V, 1, 2)/dt, [0,0,0]'];                                   % trajectory acceleration

P = P + pee0 + [0;0.2;0];
p_ref = P';
pd_ref = V';
pdd_ref = A';

for i = 1 : size(P,2)
    o_ref(i,1:3) = oee0;
    od(i,1:3) = [0,0,0];
    w_ref(1:3,i) = zyxE(o_ref(i,:))*od(i,:)';
end

save('reference_8_traj_55s.mat', 'o_ref', 'od', 'p_ref', 'pd_ref', 'pdd_ref', 'w_ref')