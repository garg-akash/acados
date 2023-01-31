clear
clc
close all


set(0, 'DefaultTextInterpreter', 'latex')
set(0, 'DefaultLegendInterpreter', 'latex')
set(0, 'DefaultAxesTickLabelInterpreter', 'latex')

lw = 2;

%% COMPARISON CARTESIAN POSITION ERROR
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);
load('data_rodyman_17-Jan-2023 09:52:53.mat')

tx_eb = 0.075;
tz_eb = 0.0275;
dt = 0.008;

p_ref(end+1:size(p_log,2),1:3) = repmat(p_ref(end,1:3),size(p_log,2)-size(p_ref,1),1);
o_ref(end+1:size(phi_log,2),1:3) = repmat(o_ref(end,1:3),size(phi_log,2)-size(o_ref,1),1);

t = 0:dt:(size(p_ref,1)-1)*dt;

for i = 1:size(p_ref,1)
    ep_norm(i) = norm(p_log(1:3,i)-p_ref(i,1:3)');
    eo_norm(i) = norm([phi_log(3,i), phi_log(2,i), phi_log(1,i)]'-o_ref(i,1:3)');
end


plot(t, ep_norm, 'k--', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(t, eo_norm, 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);

legend('$e_{p}$', '$e_{o}$');
xlabel('t [s]')
ylabel('$\mathcal{E}$ [m or rad]')
set(gca, 'FontSize',16);
%set(gca, 'XLim', [0, 5.5]);
grid on
box on
set(gcf,'color','w');
%exportgraphics(h, 'rodyman_cartesian_position_2.pdf');

%% LAMBDA
h = figure('Renderer', 'painters', 'Position', [10 10 900 300])
t = 0:dt:(size(p_ref,1))*dt;

for i = 1:16
    plot(t, lambda_log(i,:),'-', 'Color', [(i-1)/16,(i-1)/16,(i-1)/16],'linewidth',2)
    grid on
    hold on
end
plot(t, repmat(lbx(27+i), size(lambda_log,2)),'--', 'Color', [.5,.5,.5],'linewidth',2)

legend('$\lambda_1$', '$\lambda_2$', '$\lambda_3$', '$\lambda_4$', '$\lambda_5$', ...
    '$\lambda_6$', '$\lambda_7$', '$\lambda_8$', '$\lambda_9$', '$\lambda_{10}$',...
    '$\lambda_{11}$', '$\lambda_{12}$', '$\lambda_{13}$', '$\lambda_{14}$', '$\lambda_{15}$',...
    '$\lambda_{16}$', 'NumColumns', 2);

xlabel('t [s]')
ylabel('$\Lambda$')
set(gca, 'FontSize',16);
%set(gca, 'XLim', [0, 2.25]);
grid on
box on
set(gcf,'color','w');
%exportgraphics(h, 'rodyman_lambda.pdf');

%% JOINTS
h = figure('Renderer', 'painters', 'Position', [10 10 900 300])
dt = 0.009
t = 0:dt:(size(p_ref,1)-1)*dt;
nj = 9;
for i = 1:nj
    plot(t, dq_log(i,:),'-', 'Color', [(i-1)/nj,(i-1)/nj,(i-1)/nj],'linewidth',2)
    hold on
    grid on
end

for i = 1:nj    
    plot(t, repmat(ubx(2*nj+i), size(dq_log,2)),'--', 'Color', [.5,.5,.5],'linewidth',2)
    hold on
    grid on
    plot(t, repmat(lbx(2*nj+i), size(dq_log,2)),'--', 'Color', [.5,.5,.5],'linewidth',2)
end

legend('$\dot{q}_1$', '$\dot{q}_2$', '$\dot{q}_3$', '$\dot{q}_4$', '$\dot{q}_5$', ...
    '$\dot{q}_6$', '$\dot{q}_7$', '$\dot{q}_8$', '$\dot{q}_9$', 'NumColumns', 2);

xlabel('t [s]')
ylabel('$\dot{q}$ [rad/s]')
set(gca, 'FontSize',16);
set(gca, 'XLim', [0, 2.25]);
grid on
box on
set(gcf,'color','w');
%exportgraphics(h, 'rodyman_lambda.pdf');

