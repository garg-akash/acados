time = 0:dt:T;
%% Position plots
% figure
% grid on
% hold on
% plot3(p_ref(1:end,1),p_ref(1:end,2),p_ref(1:end,3),'-','linewidth',2)
% hold on
% plot3(x_sim(1,1:end),x_sim(2,1:end),x_sim(3,1:end),'--','linewidth',2)
% axis([min(x_sim(1,1:end))-0.5 max(x_sim(1,1:end))+0.5 min(x_sim(2,1:end))-0.5 max(x_sim(2,1:end))+0.5 min(x_sim(3,1:end))-0.5 max(x_sim(3,1:end))+0.5])
% legend('ref traj', 'output traj','Interpreter','latex')
% xlabel('x [m]');ylabel('y [m]');zlabel('z [m]')


pref = p_ref;
pobj = x_sim(1:3,1:end)';
% tm = linspace(0,epsilon_t*size(pref,1),size(pref,1));
% tm = linspace(0,traj_init+traj_dur+traj_last,size(pref,1));
tm = time;

figure
hold on
ax1 = subplot(3,1,1);
line2 = plot(tm,pobj(:,1),'-r','linewidth',2,'DisplayName','NMPC');
hold on
line1 = plot(tm,pref(:,1),'--k','linewidth',2,'DisplayName','ref');
grid(ax1,'on')
ylim([min(pobj(:,1))-0.25 max(pobj(:,1))+0.25])
xlabel('t [s]','Interpreter','latex');ylabel('x [m]','Interpreter','latex')
hold on

ax2 = subplot(3,1,2);
line4 = plot(tm,pobj(:,2),'-r','linewidth',2);
hold on
line3 = plot(tm,pref(:,2),'--k','linewidth',2);
grid(ax2,'on')
xlabel('t [s]','Interpreter','latex');ylabel('y [m]','Interpreter','latex')
hold on

ax3 = subplot(3,1,3);
line6 = plot(tm,pobj(:,3),'-r','linewidth',2);
hold on
lin5 = plot(tm,pref(:,3),'--k','linewidth',2);
grid(ax3,'on')
ylim([min(pobj(:,3))-0.25 max(pobj(:,3))+0.25])
xlabel('t [s]','Interpreter','latex');ylabel('z [m]','Interpreter','latex')

lg = legend(subplot(3,1,1), [line1,line2]);
lg.Location = 'northoutside';
lg.Orientation = 'horizontal';
print -depsc pref

%% orientation plots 
% figure
% grid on
% hold on
% subplot(2,2,1)
% plot(time, o_ref(1:end,1), '-', 'linewidth',2)
% hold on
% plot(time, x_sim(4,1:end),'--','linewidth',2)
% axis([0 time(end) -2 2])
% legend('Ref Euler Z', 'Output Euler Z')
% xlabel('t [s]','Interpreter','latex');ylabel('Euler angle')
% hold on
% 
% subplot(2,2,2)
% plot(time, o_ref(1:end,2), '-', 'linewidth',2)
% hold on
% plot(time, x_sim(5,1:end),'--','linewidth',2)
% axis([0 time(end) -2 2])
% legend('Ref Euler Y', 'Output Euler Y')
% xlabel('t [s]');ylabel('Euler angle')
% hold on
% 
% subplot(2,2,3)
% plot(time, o_ref(1:end,3), '-', 'linewidth',2)
% hold on
% plot(time, x_sim(6,1:end),'--','linewidth',2)
% axis([0 time(end) -2 2])
% legend('Ref Euler X', 'Output Euler X')
% xlabel('t [s]');ylabel('Euler angle')

oref = rad2deg(o_ref); %this is in order euler zyx
% oobj1 = rad2deg(o(:,4:6)); %this is in order xyz (read from simulator)
oobj = rad2deg(x_sim(4:6,:)'); %this is in order euler zyx (calculated from kinematics)

figure
hold on
ax1 = subplot(3,1,1);
line2 = plot(tm,oobj(:,1),'-r','linewidth',2,'DisplayName','NMPC');
hold on
line1 = plot(tm,oref(:,1),'--k','linewidth',2,'DisplayName','ref');
grid(ax1,'on')
ylim([min(oobj(:,1))-0.5 max(oobj(:,1))+0.5])
xlabel('t [s]','Interpreter','latex');ylabel('$Euler_z$ [deg]','Interpreter','latex')
hold on

ax2 = subplot(3,1,2);
line4 = plot(tm,oobj(:,2),'-r','linewidth',2);
hold on
line3 = plot(tm,oref(:,2),'--k','linewidth',2);
grid(ax2,'on')
ylim([min(oobj(:,2))-0.5 max(oobj(:,2))+0.5])
xlabel('t [s]','Interpreter','latex');ylabel('$Euler_y$ [deg]','Interpreter','latex')
hold on

ax3 = subplot(3,1,3);
line6 = plot(tm,oobj(:,3),'-r','linewidth',2);
hold on
lin5 = plot(tm,oref(:,3),'--k','linewidth',2);
grid(ax3,'on')
ylim([min(oobj(:,3))-0.5 max(oobj(:,3))+0.5])
xlabel('t [s]','Interpreter','latex');ylabel('$Euler_x$ [deg]','Interpreter','latex')

lg = legend(subplot(3,1,1), [line1,line2]);
lg.Location = 'northoutside';
lg.Orientation = 'horizontal';
%saveas(gcf,'oref.png')
print -depsc oref

%% Velocity plots (velocity already rotated so now in W frame)
% figure
% grid on
% hold on
% subplot(2,2,1)
% plot(time, pd_ref(1:end,1), '-','linewidth',2)
% hold on
% plot(time,pd_op(1,1:end),'--','linewidth',2)
% axis([0 time(end) -2 2])
% legend('Ref velocity x', 'Output velocity x')
% xlabel('t [s]','Interpreter','latex');ylabel('vel $[m/s]$','Interpreter','latex')
% hold on
% 
% subplot(2,2,2)
% plot(time, pd_ref(1:end,2), '-','linewidth',2)
% hold on
% plot(time,pd_op(2,1:end),'--','linewidth',2)
% axis([0 time(end) -2 2])
% legend('Ref velocity y', 'Output velocity y')
% xlabel('t [s]');ylabel('vel $[m/s]$','Interpreter','latex')
% hold on
% 
% subplot(2,2,3)
% plot(time, pd_ref(1:end,3), '-','linewidth',2)
% hold on
% plot(time,pd_op(3,1:end),'--','linewidth',2)
% axis([0 time(end) -2 2])
% legend('Ref velocity z', 'Output velocity z')
% xlabel('t [s]');ylabel('vel $[m/s]$','Interpreter','latex')

vref = pd_ref(:,1:3);
vobj = pd_op(1:3,:)';

figure
hold on
ax1 = subplot(3,1,1);
line2 = plot(tm,vobj(:,1),'-r','linewidth',2,'DisplayName','NMPC');
hold on
line1 = plot(tm,vref(:,1),'--k','linewidth',2,'DisplayName','ref');
grid(ax1,'on')
ylim([min(vobj(:,1))-0.25 max(vobj(:,1))+0.25])
xlabel('t [s]','Interpreter','latex');ylabel('$v_x$ [m/s]','Interpreter','latex')
hold on

ax2 = subplot(3,1,2);
line4 = plot(tm,vobj(:,2),'-r','linewidth',2);
hold on
line3 = plot(tm,vref(:,2),'--k','linewidth',2);
grid(ax2,'on')
xlabel('t [s]','Interpreter','latex');ylabel('$v_y$ [m/s]','Interpreter','latex')
hold on

ax3 = subplot(3,1,3);
line6 = plot(tm,vobj(:,3),'-r','linewidth',2);
hold on
lin5 = plot(tm,vref(:,3),'--k','linewidth',2);
grid(ax3,'on')
ylim([min(vobj(:,3))-0.25 max(vobj(:,3))+0.25])
xlabel('t [s]','Interpreter','latex');ylabel('$v_z$ [m/s]','Interpreter','latex')

lg = legend(subplot(3,1,1), [line1,line2]);
lg.Location = 'northoutside';
lg.Orientation = 'horizontal';
% saveas(gcf,'vref.png')
print -depsc vref

%% Angular Velocity plots
% figure
% grid on
% hold on
% subplot(2,2,1)
% plot(time, w_ref(1,1:end), '-','linewidth',2)
% hold on
% plot(time, wd_op(1,1:end),'--','linewidth',2)
% axis([0 time(end) -2 2])
% legend('Ref ang velocity x', 'Output ang velocity x')
% xlabel('t [s]','Interpreter','latex');ylabel('ang vel $[rad/s]$','Interpreter','latex')
% hold on
% 
% subplot(2,2,2)
% plot(time, w_ref(2,1:end), '-','linewidth',2)
% hold on
% plot(time, wd_op(2,1:end),'--','linewidth',2)
% axis([0 time(end) -2 2])
% legend('Ref ang velocity y', 'Output ang velocity y')
% xlabel('t [s]');ylabel('ang vel $[rad/s]$','Interpreter','latex')
% hold on
% 
% subplot(2,2,3)
% plot(time, w_ref(3,1:end), '-','linewidth',2)
% hold on
% plot(time, wd_op(3,1:end),'--','linewidth',2)
% axis([0 time(end) -2 2])
% legend('Ref ang velocity z', 'Output ang velocity z')
% xlabel('t [s]');ylabel('ang vel $[rad/s]$','Interpreter','latex')

wref = w_ref(1:3,:)';
wobj = wd_op(1:3,:)';

figure
hold on
ax1 = subplot(3,1,1);
line2 = plot(tm,wobj(:,1),'-r','linewidth',2,'DisplayName','NMPC');
hold on
line1 = plot(tm,wref(:,1),'--k','linewidth',2,'DisplayName','ref');
grid(ax1,'on')
ylim([min(wobj(:,1))-0.25 max(wobj(:,1))+0.25])
xlabel('t [s]','Interpreter','latex');ylabel('$w_x$ [rad/s]','Interpreter','latex')
hold on

ax2 = subplot(3,1,2);
line4 = plot(tm,wobj(:,2),'-r','linewidth',2);
hold on
line3 = plot(tm,wref(:,2),'--k','linewidth',2);
grid(ax2,'on')
ylim([min(wobj(:,2))-0.25 max(wobj(:,2))+0.25])
xlabel('t [s]','Interpreter','latex');ylabel('$w_y$ [rad/s]','Interpreter','latex')
hold on

ax3 = subplot(3,1,3);
line6 = plot(tm,wobj(:,3),'-r','linewidth',2);
hold on
lin5 = plot(tm,wref(:,3),'--k','linewidth',2);
grid(ax3,'on')
ylim([min(wobj(:,3))-0.25 max(wobj(:,3))+0.25])
xlabel('t [s]','Interpreter','latex');ylabel('$w_z$ [rad/s]','Interpreter','latex')

lg = legend(subplot(3,1,1), [line1,line2]);
lg.Location = 'northoutside';
lg.Orientation = 'horizontal';
%saveas(gcf,'wref.png')
print -depsc wref


%% lambda plots
% figure
% grid minor
% hold on
% subplot(4,4,1)
% % hold on
% plot(time,x_sim(14,1:end),'linewidth',2)
% axis([0 time(end) -2 2])
% xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_1$','Interpreter','latex')
% hold on
% 
% subplot(4,4,2)
% plot(time,x_sim(15,1:end),'linewidth',2)
% axis([0 time(end) -2 2])
% xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_2$','Interpreter','latex')
% hold on
% 
% subplot(4,4,3)
% plot(time,x_sim(16,1:end),'linewidth',2)
% axis([0 time(end) -2 2])
% xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_3$','Interpreter','latex')
% hold on
% 
% subplot(4,4,4)
% plot(time,x_sim(17,1:end),'linewidth',2)
% axis([0 time(end) -2 2])
% xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_4$','Interpreter','latex')
% hold on
% 
% subplot(4,4,5)
% plot(time,x_sim(18,1:end),'linewidth',2)
% axis([0 time(end) -2 2])
% xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_5$','Interpreter','latex')
% hold on
% 
% subplot(4,4,6)
% plot(time,x_sim(19,1:end),'linewidth',2)
% axis([0 time(end) -2 2])
% xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_6$','Interpreter','latex')
% hold on
% 
% subplot(4,4,7)
% plot(time,x_sim(20,1:end),'linewidth',2)
% axis([0 time(end) -2 2])
% xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_7$','Interpreter','latex')
% hold on
% 
% subplot(4,4,8)
% plot(time,x_sim(21,1:end),'linewidth',2)
% axis([0 time(end) -2 2])
% xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_8$','Interpreter','latex')
% hold on
% 
% subplot(4,4,9)
% plot(time,x_sim(22,1:end),'linewidth',2)
% axis([0 time(end) -2 2])
% xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_9$','Interpreter','latex')
% hold on
% 
% subplot(4,4,10)
% plot(time,x_sim(23,1:end),'linewidth',2)
% axis([0 time(end) -2 2])
% xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_{10}$','Interpreter','latex')
% hold on
% 
% subplot(4,4,11)
% plot(time,x_sim(24,1:end),'linewidth',2)
% axis([0 time(end) -2 2])
% xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_{11}$','Interpreter','latex')
% hold on
% 
% subplot(4,4,12)
% plot(time,x_sim(25,1:end),'linewidth',2)
% axis([0 time(end) -2 2])
% xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_{12}$','Interpreter','latex')
% hold on
% 
% subplot(4,4,13)
% plot(time,x_sim(26,1:end),'linewidth',2)
% axis([0 time(end) -2 2])
% xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_{13}$','Interpreter','latex')
% hold on
% 
% subplot(4,4,14)
% plot(time,x_sim(27,1:end),'linewidth',2)
% axis([0 time(end) -2 2])
% xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_{14}$','Interpreter','latex')
% hold on
% 
% subplot(4,4,15)
% plot(time,x_sim(28,1:end),'linewidth',2)
% axis([0 time(end) -2 2])
% xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_{15}$','Interpreter','latex')
% hold on
% 
% subplot(4,4,16)
% plot(time,x_sim(29,1:end),'linewidth',2)
% axis([0 time(end) -2 2])
% xlabel('t [s]','Interpreter','latex');ylabel('$\lambda_{16}$','Interpreter','latex')

lam_lim = 0;
lam = x_sim(14:29,:)';

figure
hold on
ax1 = subplot(4,4,1)
line1 = plot(tm,lam(:,1),'-','linewidth',2,'DisplayName','NMPC')
hold on
line2 = plot(tm,repmat(lam_lim,size(lam(:,1))),'--k','linewidth',2,'DisplayName','$\Lambda_{lim}$')
grid(ax1,'on')
xlabel('t [s]','Interpreter','latex');ylabel('$\Lambda_1$','Interpreter','latex')
hold on

ax2 = subplot(4,4,2)
plot(tm,lam(:,2),'-','linewidth',2)
hold on
plot(tm,repmat(lam_lim,size(lam(:,1))),'--k','linewidth',2,'DisplayName','$\Lambda_{lb}$')
grid(ax2,'on')
xlabel('t [s]','Interpreter','latex');ylabel('$\Lambda_2$','Interpreter','latex')
hold on

ax3 = subplot(4,4,3)
plot(tm,lam(:,3),'-','linewidth',2)
hold on
plot(tm,repmat(lam_lim,size(lam(:,1))),'--k','linewidth',2,'DisplayName','$\Lambda_{lb}$')
grid(ax3,'on')
xlabel('t [s]','Interpreter','latex');ylabel('$\Lambda_3$','Interpreter','latex')
hold on

ax4 = subplot(4,4,4)
plot(tm,lam(:,4),'-','linewidth',2)
hold on
plot(tm,repmat(lam_lim,size(lam(:,1))),'--k','linewidth',2,'DisplayName','$\Lambda_{lb}$')
grid(ax4,'on')
xlabel('t [s]','Interpreter','latex');ylabel('$\Lambda_4$','Interpreter','latex')
hold on

ax5 = subplot(4,4,5)
plot(tm,lam(:,5),'-','linewidth',2)
hold on
plot(tm,repmat(lam_lim,size(lam(:,1))),'--k','linewidth',2,'DisplayName','$\Lambda_{lb}$')
grid(ax5,'on')
xlabel('t [s]','Interpreter','latex');ylabel('$\Lambda_5$','Interpreter','latex')
hold on

ax6 = subplot(4,4,6)
plot(tm,lam(:,6),'-','linewidth',2)
hold on
plot(tm,repmat(lam_lim,size(lam(:,1))),'--k','linewidth',2,'DisplayName','$\Lambda_{lb}$')
grid(ax6,'on')
xlabel('t [s]','Interpreter','latex');ylabel('$\Lambda_6$','Interpreter','latex')
hold on

ax7 = subplot(4,4,7)
plot(tm,lam(:,7),'-','linewidth',2)
hold on
plot(tm,repmat(lam_lim,size(lam(:,1))),'--k','linewidth',2,'DisplayName','$\Lambda_{lb}$')
grid(ax7,'on')
xlabel('t [s]','Interpreter','latex');ylabel('$\Lambda_7$','Interpreter','latex')
hold on

ax8 = subplot(4,4,8)
plot(tm,lam(:,8),'-','linewidth',2)
hold on
plot(tm,repmat(lam_lim,size(lam(:,1))),'--k','linewidth',2,'DisplayName','$\Lambda_{lb}$')
grid(ax8,'on')
xlabel('t [s]','Interpreter','latex');ylabel('$\Lambda_8$','Interpreter','latex')
hold on

ax9 = subplot(4,4,9)
plot(tm,lam(:,9),'-','linewidth',2)
hold on
plot(tm,repmat(lam_lim,size(lam(:,1))),'--k','linewidth',2,'DisplayName','$\Lambda_{lb}$')
grid(ax9,'on')
xlabel('t [s]','Interpreter','latex');ylabel('$\Lambda_9$','Interpreter','latex')
hold on

ax10 = subplot(4,4,10)
plot(tm,lam(:,10),'-','linewidth',2)
hold on
plot(tm,repmat(lam_lim,size(lam(:,1))),'--k','linewidth',2,'DisplayName','$\Lambda_{lb}$')
grid(ax10,'on')
xlabel('t [s]','Interpreter','latex');ylabel('$\Lambda_{10}$','Interpreter','latex')
hold on

ax11 = subplot(4,4,11)
plot(tm,lam(:,11),'-','linewidth',2)
hold on
plot(tm,repmat(lam_lim,size(lam(:,1))),'--k','linewidth',2,'DisplayName','$\Lambda_{lb}$')
grid(ax11,'on')
xlabel('t [s]','Interpreter','latex');ylabel('$\Lambda_{11}$','Interpreter','latex')
hold on

ax12 = subplot(4,4,12)
plot(tm,lam(:,12),'-','linewidth',2)
hold on
plot(tm,repmat(lam_lim,size(lam(:,1))),'--k','linewidth',2,'DisplayName','$\Lambda_{lb}$')
grid(ax12,'on')
xlabel('t [s]','Interpreter','latex');ylabel('$\Lambda_{12}$','Interpreter','latex')
hold on

ax13 = subplot(4,4,13)
plot(tm,lam(:,13),'-','linewidth',2)
hold on
plot(tm,repmat(lam_lim,size(lam(:,1))),'--k','linewidth',2,'DisplayName','$\Lambda_{lb}$')
grid(ax13,'on')
xlabel('t [s]','Interpreter','latex');ylabel('$\Lambda_{13}$','Interpreter','latex')
hold on

ax14 = subplot(4,4,14)
plot(tm,lam(:,14),'-','linewidth',2)
hold on
plot(tm,repmat(lam_lim,size(lam(:,1))),'--k','linewidth',2,'DisplayName','$\Lambda_{lb}$')
grid(ax14,'on')
xlabel('t [s]','Interpreter','latex');ylabel('$\Lambda_{14}$','Interpreter','latex')
hold on

ax15 = subplot(4,4,15)
plot(tm,lam(:,15),'-','linewidth',2)
hold on
plot(tm,repmat(lam_lim,size(lam(:,1))),'--k','linewidth',2,'DisplayName','$\Lambda_{lb}$')
grid(ax15,'on')
xlabel('t [s]','Interpreter','latex');ylabel('$\Lambda_{15}$','Interpreter','latex')
hold on

ax16 = subplot(4,4,16)
plot(tm,lam(:,16),'-','linewidth',2)
hold on
plot(tm,repmat(lam_lim,size(lam(:,1))),'--k','linewidth',2,'DisplayName','$\Lambda_{lb}$')
grid(ax16,'on')
xlabel('t [s]','Interpreter','latex');ylabel('$\Lambda_{16}$','Interpreter','latex')

lg = legend(subplot(4,4,2), [line2,line1],'Interpreter','latex');
lg.Location = 'northoutside';
lg.Orientation = 'horizontal';
lg.Position(1) = 0.5 - lg.Position(3)/2; 
lg.Position(2) = 1-lg.Position(4);%0.5 - lg.Position(4)/2;
%saveas(gcf,'lam_from_fc_read_long_way.png')
print -depsc lam


%% Torque plots
% figure
% numSteps = size(logger.tau,2);
% dT = (1:numSteps)*dt;
% plot(dT,logger.tau(:,1:numSteps),'linewidth',2),grid on;
% legend('joint 1','joint 2','joint 3','joint 4','joint 5','joint 6','joint 7','Orientation','horizontal','Location','northoutside'	,'Interpreter','latex');
% ylabel('Joint torque [N-m]','Interpreter','latex');
% xlabel('Time [s]','Interpreter','latex');
% title('Joint Torque plot')

numSteps = size(logger.tau,2);
dT = (1:numSteps)*dt;
tau = logger.tau(:,1:numSteps)';
tau_lim = params.tau_bounds(1);
%ref 
figure('Position', [10 10 650 450])
grid on
hold on
plot(dT,tau(:,1),'-','linewidth',2)
hold on
plot(dT,tau(:,2),'-','linewidth',2)
hold on
plot(dT,tau(:,3),'-','linewidth',2)
hold on
plot(dT,tau(:,4),'-','linewidth',2)
hold on
plot(dT,tau(:,5),'-','linewidth',2)
hold on
plot(dT,tau(:,6),'-','linewidth',2)
hold on
plot(dT,tau(:,7),'-','linewidth',2)
hold on
plot(dT,repmat(tau_lim,numSteps),'--k','linewidth',2)
hold on
plot(dT,repmat(-tau_lim,numSteps),'--k','linewidth',2)
lg = legend('joint 1','joint 2','joint 3','joint 4','joint 5','joint 6','joint 7','$\tau_{lim}$','Orientation','horizontal','Location','northoutside'	,'Interpreter','latex');
lg.Position(1) = 0.5 - lg.Position(3)/2; 
lg.Position(2) = 1-lg.Position(4);%0.5 - lg.Position(4)/2;
xlabel('t [s]','Interpreter','latex');ylabel('$\tau\;[N-m]$','Interpreter','latex')
% saveas(gcf,'tau.png')
print -depsc tau