clear
clc
close all

cd /home/mrslvgg/acados/examples/acados_matlab_octave/man_obj
load data_rodyman.mat

set(0, 'DefaultTextInterpreter', 'latex')
set(0, 'DefaultLegendInterpreter', 'latex')
set(0, 'DefaultAxesTickLabelInterpreter', 'latex')

lw = 2;

%% object contact forces
Fc = o(:,19:30);
Fc_read = o(:,47:59);
dt = 0.01;
t = 0:dt:size(Fc,1)*0.01;
t_min = 1.0;
t_max = 2.6;

h = figure('Renderer', 'painters', 'Position', [10 10 900 600])

%% F_C_1
subplot(2,2,1)
plot(t(t_min/dt:t_max/dt)-t_min, Fc(t_min/dt:t_max/dt,1), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(t(t_min/dt:t_max/dt)-t_min, Fc(t_min/dt:t_max/dt,2), 'k-', 'Linewidth', lw ,'Color', [0.4, 0.4, 0.4]);
plot(t(t_min/dt:t_max/dt)-t_min, Fc(t_min/dt:t_max/dt,3), 'k-', 'Linewidth', lw ,'Color', [0.7, 0.7, 0.7]);

plot(t(t_min/dt:t_max/dt)-t_min, Fc_read(t_min/dt:t_max/dt,4), 'k--', 'Linewidth', lw ,'Color', [0.1, 0.1, 0.1]);
plot(t(t_min/dt:t_max/dt)-t_min, Fc_read(t_min/dt:t_max/dt,5), 'k--', 'Linewidth', lw ,'Color', [0.3, 0.3, 0.3]);
plot(t(t_min/dt:t_max/dt)-t_min, Fc_read(t_min/dt:t_max/dt,6), 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);

hold on
legend('$x$', '$y$', '$z$', '$x$', '$y$', '$z$');

xlabel('t [s]')
ylabel('$F_{c_1}$ [N]')
set(gca, 'FontSize',16);
set(gca, 'XLim', [0, 1.5]);
grid on
box on
set(gcf,'color','w');

%% F_C_2
subplot(2,2,2)
plot(t(t_min/dt:t_max/dt)-t_min, Fc(t_min/dt:t_max/dt,4), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(t(t_min/dt:t_max/dt)-t_min, Fc(t_min/dt:t_max/dt,5), 'k-', 'Linewidth', lw ,'Color', [0.4, 0.4, 0.4]);
plot(t(t_min/dt:t_max/dt)-t_min, Fc(t_min/dt:t_max/dt,6), 'k-', 'Linewidth', lw ,'Color', [0.7, 0.7, 0.7]);

plot(t(t_min/dt:t_max/dt)-t_min, Fc_read(t_min/dt:t_max/dt,7), 'k--', 'Linewidth', lw ,'Color', [0.1, 0.1, 0.1]);
plot(t(t_min/dt:t_max/dt)-t_min, Fc_read(t_min/dt:t_max/dt,8), 'k--', 'Linewidth', lw ,'Color', [0.3, 0.3, 0.3]);
plot(t(t_min/dt:t_max/dt)-t_min, Fc_read(t_min/dt:t_max/dt,9), 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);

hold on

xlabel('t [s]')
ylabel('$F_{c_2}$ [N]')
set(gca, 'FontSize',16);
set(gca, 'XLim', [0, 1.5]);
grid on
box on
set(gcf,'color','w');

subplot(2,2,3)
plot(t(t_min/dt:t_max/dt)-t_min, Fc(t_min/dt:t_max/dt,7), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(t(t_min/dt:t_max/dt)-t_min, Fc(t_min/dt:t_max/dt,8), 'k-', 'Linewidth', lw ,'Color', [0.4, 0.4, 0.4]);
plot(t(t_min/dt:t_max/dt)-t_min, Fc(t_min/dt:t_max/dt,9), 'k-', 'Linewidth', lw ,'Color', [0.7, 0.7, 0.7]);

plot(t(t_min/dt:t_max/dt)-t_min, Fc_read(t_min/dt:t_max/dt,10), 'k--', 'Linewidth', lw ,'Color', [0.1, 0.1, 0.1]);
plot(t(t_min/dt:t_max/dt)-t_min, Fc_read(t_min/dt:t_max/dt,11), 'k--', 'Linewidth', lw ,'Color', [0.3, 0.3, 0.3]);
plot(t(t_min/dt:t_max/dt)-t_min, Fc_read(t_min/dt:t_max/dt,12), 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);

hold on

xlabel('t [s]')
ylabel('$F_{c_3}$ [N]')
set(gca, 'FontSize',16);
set(gca, 'XLim', [0, 1.5]);
grid on
box on
set(gcf,'color','w');

subplot(2,2,4)
plot(t(t_min/dt:t_max/dt)-t_min, Fc(t_min/dt:t_max/dt,10), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(t(t_min/dt:t_max/dt)-t_min, Fc(t_min/dt:t_max/dt,11), 'k-', 'Linewidth', lw ,'Color', [0.4, 0.4, 0.4]);
plot(t(t_min/dt:t_max/dt)-t_min, Fc(t_min/dt:t_max/dt,12), 'k-', 'Linewidth', lw ,'Color', [0.7, 0.7, 0.7]);

plot(t(t_min/dt:t_max/dt)-t_min, Fc_read(t_min/dt:t_max/dt,1), 'k--', 'Linewidth', lw ,'Color', [0.1, 0.1, 0.1]);
plot(t(t_min/dt:t_max/dt)-t_min, Fc_read(t_min/dt:t_max/dt,2), 'k--', 'Linewidth', lw ,'Color', [0.3, 0.3, 0.3]);
plot(t(t_min/dt:t_max/dt)-t_min, Fc_read(t_min/dt:t_max/dt,3), 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);

hold on

xlabel('t [s]')
ylabel('$F_{c_4}$ [N]')
set(gca, 'FontSize',16);
set(gca, 'XLim', [0, 1.5]);
grid on
box on
set(gcf,'color','w');

exportgraphics(h,'contact_forces.pdf');



