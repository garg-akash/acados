%% Position plots
figure
grid on
hold on
plot(p_ref(1:301,1),'-','linewidth',2)
hold on
plot(p_log(1,1:450),'--','linewidth',2)
%axis([0 t(end) -2 2])
legend('ref x', 'output x','Interpreter','latex')
xlabel('t [s]','Interpreter','latex');ylabel('x [m]','Interpreter','latex')

figure
grid on
hold on
plot(t,p_ref(1:length(t),2),'-','linewidth',2)
hold on
plot(t,xx(2,1:end-1),'--','linewidth',2)
axis([0 t(end) -2 2])
legend('ref y', 'output y','Interpreter','latex')
xlabel('t [s]');ylabel('y [m]')

figure
grid on
hold on
plot(t,p_ref(1:length(t),3),'-','linewidth',2)
hold on
plot(t,xx(3,1:end-1),'--','linewidth',2)
axis([0 t(end) -2 2])
legend('ref z', 'output z','Interpreter','latex')
xlabel('t [s]');ylabel('z [m]')

figure
grid on
hold on
plot3(p_ref(1:length(t),1),p_ref(1:length(t),2),p_ref(1:length(t),3),'-','linewidth',2)
hold on
plot3(xx(1,1:end-1),xx(2,1:end-1),xx(3,1:end-1),'--','linewidth',2)
axis([0.2 0.6 -0.6 0.6 0.2 0.6])
legend('ref traj', 'output traj','Interpreter','latex')
xlabel('x [x]');ylabel('y [m]');zlabel('z [m]')

%% Velocity plots
figure
grid on
hold on
subplot(2,2,1)
plot(t,pd_ref(1:end-1,1),'-','linewidth',2)
hold on
plot(t,xx(7,1:end-1),'--','linewidth',2)
axis([0 t(end) -2 2])
legend('Ref velocity x', 'Output velocity x')
xlabel('t [s]','Interpreter','latex');ylabel('vel $[m/s]$','Interpreter','latex')
hold on
subplot(2,2,2)
plot(t,pd_ref(1:end-1,2),'-','linewidth',2)
hold on
plot(t,xx(8,1:end-1),'--','linewidth',2)
axis([0 t(end) -2 2])
legend('Ref velocity y', 'Output velocity y')
xlabel('t [s]');ylabel('vel $[m/s]$','Interpreter','latex')
hold on
subplot(2,2,3)
plot(t,pd_ref(1:end-1,3),'-','linewidth',2)
hold on
plot(t,xx(9,1:end-1),'--','linewidth',2)
axis([0 t(end) -2 2])
legend('Ref velocity z', 'Output velocity z')
xlabel('t [s]');ylabel('vel $[m/s]$','Interpreter','latex')

%% body wrench plots
figure
grid minor
hold on
subplot(3,2,1)
% hold on
plot(t,xx(14,1:end-1),'linewidth',2)
axis([0 t(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_1$','Interpreter','latex')
hold on
subplot(3,2,2)
plot(t,xx(15,1:end-1),'linewidth',2)
axis([0 t(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_2$','Interpreter','latex')
hold on
subplot(3,2,3)
plot(t,xx(16,1:end-1),'linewidth',2)
axis([0 t(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_3$','Interpreter','latex')
hold on
subplot(3,2,4)
plot(t,xx(17,1:end-1),'linewidth',2)
axis([0 t(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_4$','Interpreter','latex')
hold on
subplot(3,2,5)
plot(t,xx(18,1:end-1),'linewidth',2)
axis([0 t(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_5$','Interpreter','latex')
hold on
subplot(3,2,6)
plot(t,xx(19,1:end-1),'linewidth',2)
axis([0 t(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_6$','Interpreter','latex')
%% dtau plots
figure 
grid minor
for i = 1:16
    hold on
    subplot(4,4,i)
    plot(t,u_cl(:,i),'linewidth',2)
    axis([0 t(end) -2 2])
    xlabel('t [s]','Interpreter','latex');ylabel('$\dot(\tau)_i$','Interpreter','latex')
end
%% lamdba plots
figure 
grid minor
for i = 1:16
    hold on
    subplot(4,4,i)
    plot(t,xx(i+13,1:end-1),'linewidth',2)
    axis([0 t(end) -2 2])
    xlabel('t [s]','Interpreter','latex');ylabel('$\dot(\tau)_i$','Interpreter','latex')
end
%%
q_output = xx(26:31,:);
dq_output = xx(32:37,:);
for i = 1:size(q_output,2)
    pee_output(i,:) = computePee(d1,d3,d5,d7,q_output(:,i));
end
%% Pee plots
figure
grid on
hold on
subplot(2,2,1)
plot(t,pee_ref(1:length(t),1),'-','linewidth',2)
hold on
plot(t,pee_output(1:end-1,1),'-','linewidth',2)
axis([0 t(end) -2 2])
legend('Ref x', 'Output x')
xlabel('t [s]');ylabel('x $[m]$','Interpreter','latex')
hold on
subplot(2,2,2)
plot(t,pee_ref(1:length(t),2),'-','linewidth',2)
hold on
plot(t,pee_output(1:end-1,2),'-','linewidth',2)
axis([0 t(end) -2 2])
legend('Ref y', 'Output y')
xlabel('t [s]');ylabel('y $[m]$','Interpreter','latex')
hold on
subplot(2,2,3)
plot(t,pee_ref(1:length(t),3),'-','linewidth',2)
hold on
plot(t,pee_output(1:end-1,3),'-','linewidth',2)
axis([0 t(end) -2 2])
legend('Ref z', 'Output z')
xlabel('t [s]');ylabel('z $[m]$','Interpreter','latex')
hold on
subplot(2,2,4)
plot3(pee_ref(1:length(t),1),pee_ref(1:length(t),2),pee_ref(1:length(t),3),'-','linewidth',2)
hold on
plot3(pee_output(1:end-1,1),pee_output(1:end-1,2),pee_output(1:end-1,3),'-','linewidth',2)
axis([0 t(end) -2 2])
legend('Ref traj', 'Output traj')
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]')
%% Joint plots
figure
grid on
hold on
subplot(3,2,1)
plot(t,q_init(1,1:end-1),'-','linewidth',2)
hold on
plot(t,q_output(1,1:end-1),'-','linewidth',2)
axis([0 t(end) -2 2])
legend('Ref', 'Output')
xlabel('t [s]');ylabel('$joint_1$ [rad]','Interpreter','latex')
hold on
subplot(3,2,2)
plot(t,q_init(2,1:end-1),'-','linewidth',2)
hold on
plot(t,q_output(2,1:end-1),'-','linewidth',2)
axis([0 t(end) -2 2])
legend('Ref', 'Output')
xlabel('t [s]');ylabel('$joint_2$ [rad]','Interpreter','latex')
hold on
subplot(3,2,3)
plot(t,q_init(3,1:end-1),'-','linewidth',2)
hold on
plot(t,q_output(3,1:end-1),'-','linewidth',2)
axis([0 t(end) -2 2])
legend('Ref', 'Output')
xlabel('t [s]');ylabel('$joint_3$ [rad]','Interpreter','latex')
hold on
subplot(3,2,4)
plot(t,q_init(4,1:end-1),'-','linewidth',2)
hold on
plot(t,q_output(4,1:end-1),'-','linewidth',2)
axis([0 t(end) -2 2])
legend('Ref', 'Output')
xlabel('t [s]');ylabel('$joint_4$ [rad]','Interpreter','latex')
hold on
subplot(3,2,5)
plot(t,q_init(5,1:end-1),'-','linewidth',2)
hold on
plot(t,q_output(5,1:end-1),'-','linewidth',2)
axis([0 t(end) -2 2])
legend('Ref', 'Output')
xlabel('t [s]');ylabel('$joint_5$ [rad]','Interpreter','latex')
hold on
subplot(3,2,6)
plot(t,q_init(6,1:end-1),'-','linewidth',2)
hold on
plot(t,q_output(6,1:end-1),'-','linewidth',2)
axis([0 t(end) -2 2])
legend('Ref', 'Output')
xlabel('t [s]');ylabel('$joint_6$ [rad]','Interpreter','latex')