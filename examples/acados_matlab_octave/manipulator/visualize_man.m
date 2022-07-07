time = 0:dt:T;
q_output = x_sim(8:14,:);
dq_output = x_sim(15:21,:);
tau_output = x_sim(1:7,:);
%% Joint plots
figure
grid on
hold on
subplot(4,2,1)
plot(time,q_ref(1,1:end),'-','linewidth',2)
hold on
plot(time, q_output(1,1:end),'--','linewidth',2)
axis([0 time(end) -2 2])
legend('Ref', 'Output')
xlabel('t [s]');ylabel('joint_1 [rad]')
hold on
subplot(4,2,2)
plot(time,q_ref(2,1:end),'-','linewidth',2)
hold on
plot(time,q_output(2,1:end),'--','linewidth',2)
axis([0 time(end) -2 2])
legend('Ref', 'Output')
xlabel('t [s]');ylabel('joint_2 [rad]')
hold on
subplot(4,2,3)
plot(time,q_ref(3,1:end),'-','linewidth',2)
hold on
plot(time,q_output(3,1:end),'--','linewidth',2)
axis([0 time(end) -2 2])
legend('Ref', 'Output')
xlabel('t [s]');ylabel('joint_3 [rad]')
hold on
subplot(4,2,4)
plot(time,q_ref(4,1:end),'-','linewidth',2)
hold on
plot(time,q_output(4,1:end),'--','linewidth',2)
axis([0 time(end) -2 2])
legend('Ref', 'Output')
xlabel('t [s]');ylabel('joint_4 [rad]')
hold on
subplot(4,2,5)
plot(time,q_ref(5,1:end),'-','linewidth',2)
hold on
plot(time,q_output(5,1:end),'--','linewidth',2)
axis([0 time(end) -2 2])
legend('Ref', 'Output')
xlabel('t [s]');ylabel('joint_5 [rad]')
hold on
subplot(4,2,6)
plot(time,q_ref(6,1:end),'-','linewidth',2)
hold on
plot(time,q_output(6,1:end),'--','linewidth',2)
axis([0 time(end) -2 2])
legend('Ref', 'Output')
xlabel('t [s]');ylabel('joint_6 [rad]')
hold on
subplot(4,2,7)
plot(time,q_ref(7,1:end),'-','linewidth',2)
hold on
plot(time,q_output(7,1:end),'--','linewidth',2)
axis([0 time(end) -2 2])
legend('Ref', 'Output')
xlabel('t [s]');ylabel('joint_7 [rad]')

%% Torque plots
figure
numSteps = size(time,2);
plot(time,tau_output(:,1:numSteps),'linewidth',2),grid on;
legend('joint 1','joint 2','joint 3','joint 4','joint 5','joint 6','joint 7','Orientation','horizontal','Location','northoutside'	,'Interpreter','latex');
ylabel('Joint torque [N-m]','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
title('Joint Torque plot')

%% Torque plots 2
figure
grid on
hold on
subplot(4,2,1)
plot(time, tau_output(1,1:end),'-','linewidth',2)
xlabel('t [s]','Interpreter','latex');ylabel('$\tau_1$','Interpreter','latex')
hold on
subplot(4,2,2)
plot(time,tau_output(2,1:end),'-','linewidth',2)
xlabel('t [s]','Interpreter','latex');ylabel('$\tau_2$','Interpreter','latex')
hold on
subplot(4,2,3)
plot(time,tau_output(3,1:end),'-','linewidth',2)
xlabel('t [s]','Interpreter','latex');ylabel('$\tau_3$','Interpreter','latex')
hold on
subplot(4,2,4)
plot(time,tau_output(4,1:end),'-','linewidth',2)
xlabel('t [s]','Interpreter','latex');ylabel('$\tau_4$','Interpreter','latex')
hold on
subplot(4,2,5)
plot(time,tau_output(5,1:end),'-','linewidth',2)
xlabel('t [s]','Interpreter','latex');ylabel('$\tau_5$','Interpreter','latex')
hold on
subplot(4,2,6)
plot(time,tau_output(6,1:end),'-','linewidth',2)
xlabel('t [s]','Interpreter','latex');ylabel('$\tau_6$','Interpreter','latex')
hold on
subplot(4,2,7)
plot(time,tau_output(7,1:end),'-','linewidth',2)
xlabel('t [s]','Interpreter','latex');ylabel('$\tau_7$','Interpreter','latex')