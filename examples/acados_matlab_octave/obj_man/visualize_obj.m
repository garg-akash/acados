time = 0:dt:T;
%% Position plots
figure
grid on
hold on
plot3(p_ref(1:end,1),p_ref(1:end,2),p_ref(1:end,3),'-','linewidth',2)
hold on
plot3(x_sim(1,1:end),x_sim(2,1:end),x_sim(3,1:end),'--','linewidth',2)
axis([min(x_sim(1,1:end))-0.5 max(x_sim(1,1:end))+0.5 min(x_sim(2,1:end))-0.5 max(x_sim(2,1:end))+0.5 min(x_sim(3,1:end))-0.5 max(x_sim(3,1:end))+0.5])
legend('ref traj', 'output traj','Interpreter','latex')
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]')

%% orientation plots 
figure
grid on
hold on
subplot(2,2,1)
plot(time, o_ref(1:end,1), '-', 'linewidth',2)
hold on
plot(time, x_sim(4,1:end),'--','linewidth',2)
axis([0 time(end) -2 2])
legend('Ref Euler Z', 'Output Euler Z')
xlabel('t [s]','Interpreter','latex');ylabel('Euler angle')
hold on

subplot(2,2,2)
plot(time, o_ref(1:end,2), '-', 'linewidth',2)
hold on
plot(time, x_sim(5,1:end),'--','linewidth',2)
axis([0 time(end) -2 2])
legend('Ref Euler Y', 'Output Euler Y')
xlabel('t [s]');ylabel('Euler angle')
hold on

subplot(2,2,3)
plot(time, o_ref(1:end,3), '-', 'linewidth',2)
hold on
plot(time, x_sim(6,1:end),'--','linewidth',2)
axis([0 time(end) -2 2])
legend('Ref Euler X', 'Output Euler X')
xlabel('t [s]');ylabel('Euler angle')

%% Velocity plots (velocity already rotated so now in W frame)
figure
grid on
hold on
subplot(2,2,1)
plot(time, pd_ref(1:end,1), '-','linewidth',2)
hold on
plot(time,pd_op(1,1:end),'--','linewidth',2)
axis([0 time(end) -2 2])
legend('Ref velocity x', 'Output velocity x')
xlabel('t [s]','Interpreter','latex');ylabel('vel $[m/s]$','Interpreter','latex')
hold on

subplot(2,2,2)
plot(time, pd_ref(1:end,2), '-','linewidth',2)
hold on
plot(time,pd_op(2,1:end),'--','linewidth',2)
axis([0 time(end) -2 2])
legend('Ref velocity y', 'Output velocity y')
xlabel('t [s]');ylabel('vel $[m/s]$','Interpreter','latex')
hold on

subplot(2,2,3)
plot(time, pd_ref(1:end,3), '-','linewidth',2)
hold on
plot(time,pd_op(3,1:end),'--','linewidth',2)
axis([0 time(end) -2 2])
legend('Ref velocity z', 'Output velocity z')
xlabel('t [s]');ylabel('vel $[m/s]$','Interpreter','latex')

%% Angular Velocity plots
figure
grid on
hold on
subplot(2,2,1)
plot(time, w_ref(1,1:end), '-','linewidth',2)
hold on
plot(time, wd_op(1,1:end),'--','linewidth',2)
axis([0 time(end) -2 2])
legend('Ref ang velocity x', 'Output ang velocity x')
xlabel('t [s]','Interpreter','latex');ylabel('ang vel $[rad/s]$','Interpreter','latex')
hold on

subplot(2,2,2)
plot(time, w_ref(2,1:end), '-','linewidth',2)
hold on
plot(time, wd_op(2,1:end),'--','linewidth',2)
axis([0 time(end) -2 2])
legend('Ref ang velocity y', 'Output ang velocity y')
xlabel('t [s]');ylabel('ang vel $[rad/s]$','Interpreter','latex')
hold on

subplot(2,2,3)
plot(time, w_ref(3,1:end), '-','linewidth',2)
hold on
plot(time, wd_op(3,1:end),'--','linewidth',2)
axis([0 time(end) -2 2])
legend('Ref ang velocity z', 'Output ang velocity z')
xlabel('t [s]');ylabel('ang vel $[rad/s]$','Interpreter','latex')

%% lambda plots
figure
grid minor
hold on
subplot(4,4,1)
% hold on
plot(time,x_sim(14,1:end),'linewidth',2)
axis([0 time(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_1$','Interpreter','latex')
hold on

subplot(4,4,2)
plot(time,x_sim(15,1:end),'linewidth',2)
axis([0 time(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_2$','Interpreter','latex')
hold on

subplot(4,4,3)
plot(time,x_sim(16,1:end),'linewidth',2)
axis([0 time(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_3$','Interpreter','latex')
hold on

subplot(4,4,4)
plot(time,x_sim(17,1:end),'linewidth',2)
axis([0 time(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_4$','Interpreter','latex')
hold on

subplot(4,4,5)
plot(time,x_sim(18,1:end),'linewidth',2)
axis([0 time(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_5$','Interpreter','latex')
hold on

subplot(4,4,6)
plot(time,x_sim(19,1:end),'linewidth',2)
axis([0 time(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_6$','Interpreter','latex')
hold on

subplot(4,4,7)
plot(time,x_sim(20,1:end),'linewidth',2)
axis([0 time(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_7$','Interpreter','latex')
hold on

subplot(4,4,8)
plot(time,x_sim(21,1:end),'linewidth',2)
axis([0 time(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_8$','Interpreter','latex')
hold on

subplot(4,4,9)
plot(time,x_sim(22,1:end),'linewidth',2)
axis([0 time(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_9$','Interpreter','latex')
hold on

subplot(4,4,10)
plot(time,x_sim(23,1:end),'linewidth',2)
axis([0 time(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_{10}$','Interpreter','latex')
hold on

subplot(4,4,11)
plot(time,x_sim(24,1:end),'linewidth',2)
axis([0 time(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_{11}$','Interpreter','latex')
hold on

subplot(4,4,12)
plot(time,x_sim(25,1:end),'linewidth',2)
axis([0 time(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_{12}$','Interpreter','latex')
hold on

subplot(4,4,13)
plot(time,x_sim(26,1:end),'linewidth',2)
axis([0 time(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_{13}$','Interpreter','latex')
hold on

subplot(4,4,14)
plot(time,x_sim(27,1:end),'linewidth',2)
axis([0 time(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_{14}$','Interpreter','latex')
hold on

subplot(4,4,15)
plot(time,x_sim(28,1:end),'linewidth',2)
axis([0 time(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_{15}$','Interpreter','latex')
hold on

subplot(4,4,16)
plot(time,x_sim(29,1:end),'linewidth',2)
axis([0 time(end) -2 2])
xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_{16}$','Interpreter','latex')

%% Torque plots
figure
plot(time(1:size(tau_log,2)),tau_log(:,1:end),'linewidth',2),grid on;
legend('joint 1','joint 2','joint 3','joint 4','joint 5','joint 6','joint 7','Orientation','horizontal','Location','northoutside'	,'Interpreter','latex');
ylabel('Joint torque [N-m]','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
title('Joint Torque plot')

% %% kkt plot
% figure
% semilogy(time(1:end-1),KKT_MPC,'linewidth',1.5,'color','k','linestyle','--','marker','.');hold on
% grid on;
% xlabel('t [s]','FontSize',13);    ylabel('MPC KKT','FontSize',13)