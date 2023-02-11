clear
clc
close all


set(0, 'DefaultTextInterpreter', 'latex')
set(0, 'DefaultLegendInterpreter', 'latex')
set(0, 'DefaultAxesTickLabelInterpreter', 'latex')

lw = 2;

%% COMPARISON CARTESIAN POSITION ERROR
h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);
load /home/mrslvgg/acados/examples/acados_matlab_octave/man_obj/reference_traj_25s.mat
cd /home/mrslvgg/Desktop/RODYMAN_EXPERIMENTS/24Giu/2.5s/0.15/REF2
tx_eb = 0.075;
tz_eb = 0.0275;
ti=0;
te=450;
p_ref(313:468,1:3) = repmat(p_ref(313,1:3),156,1);
o_ref(313:468,1:3) = repmat(o_ref(313,1:3),156,1);

t = ti:0.005:(te-ti-1)*0.005;

cart = readmatrix('cart_pos.txt');
for i = 1:size(cart,1)
    ep_norm(i) = norm(cart(i,1:3)-p_ref(i,1:3)-[tx_eb,0,-tz_eb]);
    eo_norm(i) = norm(cart(i,4:6)-o_ref(i,1:3));
end
ep_norm(313:te) = ep_norm(313);
eo_norm(313:te) = eo_norm(313);


plot(t, ep_norm, 'k--', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(t, eo_norm, 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);

cd /home/mrslvgg/Desktop/RODYMAN_EXPERIMENTS/24Giu/2.5s/0.10/LOG2
cart = readmatrix('cart_pos.txt');
clear ep_norm eo_norm

for i = 1:size(cart,1)
    ep_norm(i) = norm(cart(i,1:3)-p_ref(i,1:3)-[tx_eb,0,-tz_eb]);
    eo_norm(i) = norm(cart(i,4:6)-o_ref(i,1:3));
end
ep_norm(313:te) = ep_norm(313);
eo_norm(313:te) = eo_norm(313);

plot(t, ep_norm(1:450), 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(t, eo_norm(1:450), 'k-', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);

legend('$e_{p,r}$', '$e_{o,r}$', '$e_{p,m}$', '$e_{o,m}$');
xlabel('t [s]')
ylabel('$\mathcal{E}$ [m or rad]')
set(gca, 'FontSize',16);
set(gca, 'XLim', [0, 2.25]);
grid on
box on
set(gcf,'color','w');
%exportgraphics(h, 'rodyman_cartesian_position.pdf');


%% COMPARISON OBJECT DISPLACEMENT
h = figure('Renderer', 'painters', 'Position', [10 10 900 300])
cd /home/mrslvgg/Desktop/RODYMAN_EXPERIMENTS/24Giu/1.5s/REF1
load obj_pos.mat
ti=550;
te=ti+75;
for i = ti+1:te
    disp_norm(i-ti) = norm([obj_pos{i}.Pose.Position.X, obj_pos{i}.Pose.Position.Y, obj_pos{i}.Pose.Position.Z]-[obj_pos{ti}.Pose.Position.X, obj_pos{ti}.Pose.Position.Y, obj_pos{ti}.Pose.Position.Z]);
end
t = 0:0.0336:(te-ti-1)*0.0336;
plot(t,disp_norm, 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);
hold on
cd /home/mrslvgg/Desktop/RODYMAN_EXPERIMENTS/24Giu/1.5s/LOG1
load obj_pos.mat
clear disp_norm
ti=550;
te=ti+75;
for i = ti+1:te
    disp_norm(i-ti) = norm([obj_pos{i}.Pose.Position.X, obj_pos{i}.Pose.Position.Y, obj_pos{i}.Pose.Position.Z]-[obj_pos{ti}.Pose.Position.X, obj_pos{ti}.Pose.Position.Y, obj_pos{ti}.Pose.Position.Z]);
end
t = 0:0.0336:(te-ti-1)*0.0336;
plot(t,disp_norm, 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);

legend('$d_r$', '$d_m$');
xlabel('t [s]')
ylabel('$\mathcal{D}$ [m]')
set(gca, 'FontSize',16);
set(gca, 'XLim', [0, 2.25]);
grid on
box on
set(gcf,'color','w');
%exportgraphics(h, 'rodyman_object_displacement.pdf');

%% LAMBDA
load('/home/mrslvgg/acados/examples/acados_matlab_octave/man_obj/data_rodyman_24-Jun-2022 16:03:51________.mat')
h = figure('Renderer', 'painters', 'Position', [10 10 900 300])
ti=0;
te=450;
t = ti:0.005:(te-ti)*0.005;

for i = 1:16
    for j = 1:size(t,2)
        if(lambda_log(i,j) < 0)
            lambda_log(i,j) = 0.1*lambda_log(i,j);
        end
    end    
    plot(t, lambda_log(i,:),'-', 'Color', [(i-1)/16,(i-1)/16,(i-1)/16],'linewidth',2)
    grid on
    hold on
end
plot(t, repmat(lbx(27+i), size(lambda_log,2)),'--', 'Color', [.5,.5,.5],'linewidth',2)

l = legend('$\lambda_1$', '$\lambda_2$', '$\lambda_3$', '$\lambda_4$', '$\lambda_5$', ...
    '$\lambda_6$', '$\lambda_7$', '$\lambda_8$', '$\lambda_9$', '$\lambda_{10}$',...
    '$\lambda_{11}$', '$\lambda_{12}$', '$\lambda_{13}$', '$\lambda_{14}$', '$\lambda_{15}$',...
    '$\lambda_{16}$', 'NumColumns', 8);
set(l, 'Location', 'northeast');

xlabel('t [s]')
ylabel('$\Lambda$')
set(gca, 'FontSize',16);
set(gca, 'XLim', [0, 2.25]);
set(gca, 'YLim', [-0.1, 1]);

grid on
box on
set(gcf,'color','w');
exportgraphics(h, 'rodyman_lambda.pdf');
