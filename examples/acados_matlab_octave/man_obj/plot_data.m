close all
clc
clear

set(0, 'DefaultTextInterpreter', 'latex')
set(0, 'DefaultLegendInterpreter', 'latex')
set(0, 'DefaultAxesTickLabelInterpreter', 'latex')

lw = 2;

%% INPUT CONSTRAINT
load data.mat
p = [p;repmat(p(end,:),size(p_log,2)-size(p,1),1)];
o_wb = [o_wb;repmat(o_wb(end,:),size(phi_log,2)-size(o_wb,1),1)];
t = [0:0.01:size(p_log,2)*0.01];

%% Error norm
h = figure('Renderer', 'painters', 'Position', [10 10 900 300])
for i = 1:size(p_log,2)
    np(i) = norm([p_log(1,i) - p(i,1), ...
        p_log(2,i) - p(i,2), ...
        p_log(3,i) - p(i,3)]);
        
    no(i) = norm([phi_log(3,i) - o_wb(i,1), ...
        phi_log(2,i) - o_wb(i,2), ...
        phi_log(1,i) - o_wb(i,3)]);
end

plot(t(1:size(np,2)), np, 'k-', 'Linewidth', lw ,'Color', [0.0, 0.0, 0.0]);
hold on
plot(t(1:size(no,2)), no, 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);
hold on
%set(gca, 'YLim', [0, 0.06]);
l = legend('$e_p$', '$e_o$');

xlabel('t [s]')
ylabel('$\mathcal{E}$ [m or rad]')
set(gca, 'FontSize',16);
set(gca, 'XLim', [0, 2.25]);
grid on
box on
set(gcf,'color','w');
exportgraphics(h,'error.pdf');

%% Lambda min
for i = 1:size(lambda_log,2)
    lambda_min(i) = min(lambda_log(:,i));
end

h = figure('Renderer', 'painters', 'Position', [10 10 900 300])
plot(t(1:size(lambda_min,2)), lambda_min, 'k-', 'Linewidth', lw ,'Color', [0.0, 0.0, 0.0])
hold on
plot(t(1:size(lambda_min,2)), zeros(size(lambda_min,2)), 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5])

xlabel('t [s]')
ylabel('$\Lambda_{min}$')
set(gca, 'FontSize',16);
set(gca, 'YLim', [min(lambda_min), max(lambda_min)]);
set(gca, 'XLim', [0, 2.25]);
grid on
box on
set(gcf,'color','w');

exportgraphics(h,'lambda_min.pdf');

%% Lambda
h = figure('Renderer', 'painters', 'Position', [10 10 900 300])

for i = 1:16
    plot(t(1:size(lambda_log,2)), lambda_log(i,:), 'k-', 'Linewidth', lw ,'Color', [(i-1)*0.05, (i-1)*0.05, (i-1)*0.05])
    hold on
end
plot(t(1:size(lambda_min,2)), zeros(size(lambda_min,2)), 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5])
l = legend('$\lambda_1$', '$\lambda_2$', ...
    '$\lambda_3$', '$\lambda_4$', ... 
    '$\lambda_5$', '$\lambda_6$', ... 
    '$\lambda_7$', '$\lambda_8$', ...
    '$\lambda_9$', '$\lambda_{10}$', ...
    '$\lambda_{11}$', '$\lambda_{12}$', ... 
    '$\lambda_{13}$', '$\lambda_{14}$', ... 
    '$\lambda_{15}$', '$\lambda_{16}$');
set(l, 'Location', 'east', 'NumColumns',2);
xlabel('t [s]')
ylabel('$\Lambda$')
set(gca, 'FontSize',16);
set(gca, 'YLim', [-0.2, max(max(lambda_log))]);
set(gca, 'XLim', [0, 2.25]);
grid on
box on
set(gcf,'color','w');

exportgraphics(h,'lambda.pdf');

%% u
h = figure('Renderer', 'painters', 'Position', [10 10 900 300])
plot(t(1:size(dtau_log,2)), dtau_log(1,:), 'k-', 'Linewidth', lw ,'Color', [0.0, 0.0, 0.0])
hold on
plot(t(1:size(dtau_log,2)), dtau_log(2,:), 'k-', 'Linewidth', lw ,'Color', [0.1, 0.1, 0.1])
hold on
plot(t(1:size(dtau_log,2)), dtau_log(3,:), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2])
hold on
plot(t(1:size(dtau_log,2)), dtau_log(4,:), 'k-', 'Linewidth', lw ,'Color', [0.3, 0.3, 0.3])
hold on
plot(t(1:size(dtau_log,2)), dtau_log(5,:), 'k-', 'Linewidth', lw ,'Color', [0.4, 0.4, 0.4])
hold on
plot(t(1:size(dtau_log,2)), dtau_log(6,:), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5])
hold on
plot(t(1:size(dtau_log,2)), dtau_log(7,:), 'k-', 'Linewidth', lw ,'Color', [0.6, 0.6, 0.6])
hold on
plot(t(1:size(dtau_log,2)), lbu(1)*ones(size(dtau_log,2)), 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5])
hold on
plot(t(1:size(dtau_log,2)), ubu(1)*ones(size(dtau_log,2)), 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5])

xlabel('t [s]')
ylabel('$\dot{\tau}$')
set(gca, 'FontSize', 16);
set(gca, 'YLim', [lbu(1), ubu(1)]);
set(gca, 'XLim', [0, 2.25]);
grid on
box on
l = legend('$\dot{\tau}_1$', '$\dot{\tau}_2$', ...
    '$\dot{\tau}_3$', '$\dot{\tau}_4$', ... 
    '$\dot{\tau}_5$', '$\dot{\tau}_6$', ... 
    '$\dot{\tau}_7$');
set(l, 'Location', 'east');
set(gcf,'color','w');

exportgraphics(h,'tau_dot.pdf');

clearvars -except lw

%% STATE CONSTRAINT
load data_jl.mat
p = [p;repmat(p(end,:),size(p_log,2)-size(p,1),1)];
o_wb = [o_wb;repmat(o_wb(end,:),size(phi_log,2)-size(o_wb,1),1)];
t = [0:0.01:size(p_log,2)*0.01];

%% Error norm
h = figure('Renderer', 'painters', 'Position', [10 10 900 300])
for i = 1:size(p_log,2)
    np(i) = norm([p_log(1,i) - p(i,1), ...
        p_log(2,i) - p(i,2), ...
        p_log(3,i) - p(i,3)]);
        
    no(i) = norm([phi_log(3,i) - o_wb(i,1), ...
        phi_log(2,i) - o_wb(i,2), ...
        phi_log(1,i) - o_wb(i,3)]);
end

plot(t(1:size(np,2)), np, 'k-', 'Linewidth', lw ,'Color', [0.0, 0.0, 0.0]);
hold on
plot(t(1:size(no,2)), no, 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);
hold on
%set(gca, 'YLim', [0, 0.06]);
set(gca, 'XLim', [0, 2.25]);
legend('$e_p$', '$e_o$');

xlabel('t [s]')
ylabel('$\mathcal{E}$ [m or rad]')
set(gca, 'FontSize',16);
grid on
box on
set(gcf,'color','w');
exportgraphics(h,'error_jl.pdf');

%% Lambda min
for i = 1:size(lambda_log,2)
    lambda_min(i) = min(lambda_log(:,i));
end

h = figure('Renderer', 'painters', 'Position', [10 10 900 300])
plot(t(1:size(lambda_min,2)), lambda_min, 'k-', 'Linewidth', lw ,'Color', [0.0, 0.0, 0.0])
hold on
plot(t(1:size(lambda_min,2)), zeros(size(lambda_min,2)), 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5])

xlabel('t [s]')
ylabel('$\Lambda_{min}$')
set(gca, 'FontSize',16);
set(gca, 'YLim', [min(lambda_min), max(lambda_min)]);
set(gca, 'XLim', [0, 2.25]);
grid on
box on
set(gcf,'color','w');

exportgraphics(h,'lambda_min_jl.pdf');

%% Lambda
h = figure('Renderer', 'painters', 'Position', [10 10 900 300])

for i = 1:16
    plot(t(1:size(lambda_log,2)), lambda_log(i,:), 'k-', 'Linewidth', lw ,'Color', [(i-1)*0.05, (i-1)*0.05, (i-1)*0.05])
    hold on
end
plot(t(1:size(lambda_min,2)), zeros(size(lambda_min,2)), 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5])
l = legend('$\lambda_1$', '$\lambda_2$', ...
    '$\lambda_3$', '$\lambda_4$', ... 
    '$\lambda_5$', '$\lambda_6$', ... 
    '$\lambda_7$', '$\lambda_8$', ...
    '$\lambda_9$', '$\lambda_{10}$', ...
    '$\lambda_{11}$', '$\lambda_{12}$', ... 
    '$\lambda_{13}$', '$\lambda_{14}$', ... 
    '$\lambda_{15}$', '$\lambda_{16}$');
set(l, 'Location', 'east', 'NumColumns',2);
xlabel('t [s]')
ylabel('$\Lambda$')
set(gca, 'FontSize',16);
set(gca, 'YLim', [-0.2, max(max(lambda_log))]);
set(gca, 'XLim', [0, 2.25]);
grid on
box on
set(gcf,'color','w');

exportgraphics(h,'lambda_jl.pdf');

%% q_dot
h = figure('Renderer', 'painters', 'Position', [10 10 900 300])
for i = 1:7
    plot(t(1:size(dq_log,2)), dq_log(i,:), 'k-', 'Linewidth', lw ,'Color', [(i-1)*0.1, (i-1)*0.1, (i-1)*0.1])
    hold on
end
for i = 1:7
    plot(t(1:size(dq_log,2)), lbx(14+i)*ones(size(dq_log,2)), 'k--', 'Linewidth', lw ,'Color', [(i-1)*0.1, (i-1)*0.1, (i-1)*0.1])
    plot(t(1:size(dq_log,2)), ubx(14+i)*ones(size(dq_log,2)), 'k--', 'Linewidth', lw ,'Color', [(i-1)*0.1, (i-1)*0.1, (i-1)*0.1])
end
xlabel('t [s]')
ylabel('$\dot{q}$')
set(gca, 'FontSize',16);
%set(gca, 'YLim', [lbx(14+4), ubx(14+4)]);
set(gca, 'XLim', [0, 2.25]);
grid on
box on
set(gcf,'color','w');

l = legend('$\dot{q}_1$', '$\dot{q}_2$', ...
    '$\dot{q}_3$', '$\dot{q}_4$', ... 
    '$\dot{q}_5$', '$\dot{q}_6$', ... 
    '$\dot{q}_7$');

set(l, 'Location', 'east');
exportgraphics(h,'q_dot_jl.pdf')

%% q_dot 4
h = figure('Renderer', 'painters', 'Position', [10 10 900 300])
plot(t(1:size(dq_log,2)), dq_log(4,:), 'k-', 'Linewidth', lw ,'Color', [0.0, 0.0, 0.0])
hold on
plot(t(1:size(dq_log,2)), lbx(14+4)*ones(size(dq_log,2)), 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5])
plot(t(1:size(dq_log,2)), ubx(14+4)*ones(size(dq_log,2)), 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5])

xlabel('t [s]')
ylabel('$\dot{q}_{4}$')
set(gca, 'FontSize',16);
set(gca, 'YLim', [lbx(14+4), ubx(14+4)]);
set(gca, 'XLim', [0, 2.25]);
grid on
box on
set(gcf,'color','w');

exportgraphics(h,'q_dot_4_jl.pdf')

%%
clearvars -except lw

%% NO CONSTRAINTS
load data_noconstr.mat
p = [p;repmat(p(end,:),size(p_log,2)-size(p,1),1)];
o_wb = [o_wb;repmat(o_wb(end,:),size(phi_log,2)-size(o_wb,1),1)];
t = [0:0.01:size(p_log,2)*0.01];

%% Error norm
h = figure('Renderer', 'painters', 'Position', [10 10 900 300])
for i = 1:size(p_log,2)
    np(i) = norm([p_log(1,i) - p(i,1), ...
        p_log(2,i) - p(i,2), ...
        p_log(3,i) - p(i,3)]);
        
    no(i) = norm([phi_log(3,i) - o_wb(i,1), ...
        phi_log(2,i) - o_wb(i,2), ...
        phi_log(1,i) - o_wb(i,3)]);
end

plot(t(1:size(np,2)), np, 'k-', 'Linewidth', lw ,'Color', [0.0, 0.0, 0.0]);
hold on
plot(t(1:size(no,2)), no, 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);
hold on
%set(gca, 'YLim', [0, 0.06]);
set(gca, 'XLim', [0, 2.5]);
legend('$e_p$', '$e_o$');

xlabel('t [s]')
ylabel('$\mathcal{E}$ [m or rad]')
set(gca, 'FontSize',16);
grid on
box on
set(gcf,'color','w');
exportgraphics(h,'error_noconstr.pdf');

clearvars -except lw

%% DISTURBANCE TORQUE
load data_dist_20N_1s_25s_traj.mat
p = [p;repmat(p(end,:),size(p_log,2)-size(p,1),1)];
o_wb = [o_wb;repmat(o_wb(end,:),size(phi_log,2)-size(o_wb,1),1)];
t = [0:0.01:size(p_log,2)*0.01];

%% Error norm
h = figure('Renderer', 'painters', 'Position', [10 10 900 300])
for i = 1:size(p_log,2)
    np(i) = norm([p_log(1,i) - p(i,1), ...
        p_log(2,i) - p(i,2), ...
        p_log(3,i) - p(i,3)]);
        
    no(i) = norm([phi_log(3,i) - o_wb(i,1), ...
        phi_log(2,i) - o_wb(i,2), ...
        phi_log(1,i) - o_wb(i,3)]);
end

plot(t(1:size(np,2)), np, 'k-', 'Linewidth', lw ,'Color', [0.0, 0.0, 0.0]);
hold on
plot(t(1:size(no,2)), no, 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);
hold on
%set(gca, 'YLim', [0, 0.06]);
set(gca, 'XLim', [0, 2.5]);
legend('$e_p$', '$e_o$');

xlabel('t [s]')
ylabel('$\mathcal{E}$ [m or rad]')
set(gca, 'FontSize',16);
grid on
box on
set(gcf,'color','w');
exportgraphics(h,'error_dist.pdf');

%% Lambda
h = figure('Renderer', 'painters', 'Position', [10 10 900 300])

for i = 1:16
    plot(t(1:size(lambda_log,2)), lambda_log(i,:), 'k-', 'Linewidth', lw ,'Color', [(i-1)*0.05, (i-1)*0.05, (i-1)*0.05])
    hold on
end
plot(t(1:size(lambda_log,2)), zeros(size(lambda_log,2)), 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5])
l = legend('$\lambda_1$', '$\lambda_2$', ...
    '$\lambda_3$', '$\lambda_4$', ... 
    '$\lambda_5$', '$\lambda_6$', ... 
    '$\lambda_7$', '$\lambda_8$', ...
    '$\lambda_9$', '$\lambda_{10}$', ...
    '$\lambda_{11}$', '$\lambda_{12}$', ... 
    '$\lambda_{13}$', '$\lambda_{14}$', ... 
    '$\lambda_{15}$', '$\lambda_{16}$');
set(l, 'Location', 'east', 'NumColumns',2);
xlabel('t [s]')
ylabel('$\Lambda$')
set(gca, 'FontSize',16);
set(gca, 'YLim', [-0.2, max(max(lambda_log))]);
set(gca, 'XLim', [0, 2.25]);
grid on
box on
set(gcf,'color','w');

exportgraphics(h,'lambda_dist.pdf');

%% u
h = figure('Renderer', 'painters', 'Position', [10 10 900 300])
plot(t(1:size(dtau_log,2)), dtau_log(1,:), 'k-', 'Linewidth', lw ,'Color', [0.0, 0.0, 0.0])
hold on
plot(t(1:size(dtau_log,2)), dtau_log(2,:), 'k-', 'Linewidth', lw ,'Color', [0.1, 0.1, 0.1])
hold on
plot(t(1:size(dtau_log,2)), dtau_log(3,:), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2])
hold on
plot(t(1:size(dtau_log,2)), dtau_log(4,:), 'k-', 'Linewidth', lw ,'Color', [0.3, 0.3, 0.3])
hold on
plot(t(1:size(dtau_log,2)), dtau_log(5,:), 'k-', 'Linewidth', lw ,'Color', [0.4, 0.4, 0.4])
hold on
plot(t(1:size(dtau_log,2)), dtau_log(6,:), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5])
hold on
plot(t(1:size(dtau_log,2)), dtau_log(7,:), 'k-', 'Linewidth', lw ,'Color', [0.6, 0.6, 0.6])
hold on
plot(t(1:size(dtau_log,2)), lbu(1)*ones(size(dtau_log,2)), 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5])
hold on
plot(t(1:size(dtau_log,2)), ubu(1)*ones(size(dtau_log,2)), 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5])

xlabel('t [s]')
ylabel('$\dot{\tau}$')
set(gca, 'FontSize', 16);
set(gca, 'YLim', [lbu(1), ubu(1)]);
set(gca, 'XLim', [0, 2.25]);
grid on
box on
l = legend('$\dot{\tau}_1$', '$\dot{\tau}_2$', ...
    '$\dot{\tau}_3$', '$\dot{\tau}_4$', ... 
    '$\dot{\tau}_5$', '$\dot{\tau}_6$', ... 
    '$\dot{\tau}_7$');
set(l, 'Location', 'east');
set(gcf,'color','w');

exportgraphics(h,'tau_dot_dist.pdf');


