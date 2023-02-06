side = 0.2;

pee0 = [0 0 0]';%T0_we.t; 
pee1 = [0 side 0]';
pee2 = [0.5*side side 0]';
pee3 = [0.5*side 0 0]';
pee4 = [0 0 0]';

eul = rotm2eul(T0_we.R,'ZYX')';
oee0 = eul; %initial euler zyx of ee wrt world

%%%%%
%% TRAJECTORY
dt = 0.008;
                                                           
traj = mstraj([pee0';pee1';pee2';pee3';pee4'], [], [1.8,0.9,1.8,0.9], [], dt, 0.2, zeros(1,3), zeros(1,3));

T = size(traj,1)*dt;

P = [traj(1,:); traj];
V = [diff(P, 1, 1)/dt; [0,0,0]];                                   % trajectory velocity
A = [diff(V, 1, 1)/dt; [0,0,0]];                                   % trajectory acceleration

P = P + T0_we.t' + [0;0.3;0]';
p_ref = P;
pd_ref = V;
pdd_ref = A;

for i = 1 : size(P)
    o_ref(i,1:3) = oee0;
    od(i,1:3) = [0,0,0];
    w_ref(1:3,i) = zyxE(o_ref(i,:))*od(i,:)';
end

%figure; plot3(traj(:,1), traj(:,2), traj(:,3));
%axis equal; grid on;
%figure; plot(V); grid on;
%figure; plot(A); grid on;

%save('reference_square_traj.mat', 'o_ref', 'od', 'p_ref', 'pd_ref', 'pdd_ref', 'w_ref')