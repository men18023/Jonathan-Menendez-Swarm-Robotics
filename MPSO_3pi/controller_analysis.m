% C�digo basado en el de Aldo, pues se quer�a comparar sus gr�ficas del PSO
% con las gr�ficas del ACO.

load('analysis.mat')
%t = 32*linspace(0, length(v_hist)-1, length(v_hist));
controlador = 31;
% % Velocidad lineal de Robot ------------------------
% h2 = figure(1);
% ID = 1;
% str = 'Velocidad lineal';
% set(h2,'units','points','position',[60,95,620,420],'name',str);
% set(h2,'color','w');
% plot(t, v_hist, 'LineWidth', 2, 'Color', [.1 .4 .7]);
% xlim([0, t(end)]);
% grid on; grid minor;
% xlabel('Tiempo (ms)', 'FontSize', 16); ylabel('Velocidad (m/s)', 'FontSize', 16);
% lgd = legend('Velocidad lineal de robot', 'Location', 'best');
% lgd.FontSize = 14;
% 
% %% Velocidad angular de Robot ------------------
% h3 = figure(2);
% str = 'Velocidad lineal';
% set(h3, 'units', 'points', 'position', [60,95,620,420], 'name', str);
% set(h3, 'color', 'w');
% plot(t, w_hist, 'LineWidth', 2, 'Color', [.9 .8 .1]);
% xlim([0, t(end)]);
% grid on; grid minor;
% xlabel('Tiempo (ms)','FontSize',16); ylabel('Velocidad (rad/s)','FontSize',16);
% lgd = legend('Velocidad angular de robot','Location','best');
% lgd.FontSize = 14;
% 
% %% Velocidades angulares de Robot -------------
% h4 = figure(3);
% str = 'Velocidad lineal';
% set(h4,'units','points','position',[60,95,620,420],'name',str);
% set(h4,'color','w');
% ax = axes('Parent',h4,'Position',[0.0623716632443532 0.105031948881789 0.907765757474463 0.870952303086714]);
% hold(ax,'on');
% plot(t, rwheel_hist,'LineWidth',2,'Color',[.1 .7 .3]);
% hold on;
% plot(t, lwheel_hist,'LineWidth',2,'Color',[.6 .1 .4]);
% hold on;
% plot([0 t(end)],[6.28 6.28],'LineStyle','--','Color','k');
% hold on;
% plot([0 t(end)],[-6.28 -6.28],'LineStyle','--','Color','k');
% xlim([0,t(end)]);
% grid on; grid minor;
% xlabel('Time (ms)','FontSize',16); ylabel('Velocity (rad/s)','FontSize',16);
% lgd = legend('Velocidad motor derecho','Velocidad motor izquierdo','Location','best');
% lgd.FontSize = 14;
% box(ax,'on');
% grid(ax,'on');
% set(ax,'XMinorGrid','on','YMinorGrid','on','ZMinorGrid','on');
% % set(lgd,'Position',[0.0672128188013608 0.112952606873517 0.366543347512945 0.104832270655769],'FontSize',14);

%% Trayectoria con PSO -----------------
% Create a figure
% Create a figure
h5 = figure(4);
str = 'Velocidad lineal';
set(h5, 'units', 'points', 'position', [60, 95, 520, 420], 'name', str);

% Define custom colors for the trajectories
trajectory_colors = lines(3);

% Plot trajectories with custom line styles and colors
plot(trajectory{1}(2:end, 1), trajectory{1}(2:end, 2), 'LineWidth', 2, 'Color', trajectory_colors(1, :), 'LineStyle', '-');
hold on;
plot(trajectory{2}(2:end, 1), trajectory{2}(2:end, 2), 'LineWidth', 2, 'Color', trajectory_colors(2, :), 'LineStyle', '-');
plot(trajectory{3}(2:end, 1), trajectory{3}(2:end, 2), 'LineWidth', 2, 'Color', trajectory_colors(3, :), 'LineStyle', '-');

% Customize the view
view(0, 90);

% Plot the goal as a filled black circle
hold on;
scatter(goal(1), goal(2), 200, 'k', 'filled');

% Set axis limits and grid
xlim([-1.5 1.5]);
ylim([-2 2]);
grid on;
grid minor;

% Label the axes
xlabel('X (m)');
ylabel('Y (m)');

% Add a legend with labels for the trajectories
legend('Agente 1', 'Agente 2', 'Agente 3', '{\it Global Best}');

% Set the background color of the figure to white
set(h5, 'color', 'w');

% Add a title
%title('Trajectories of Agents');

% You can also adjust the font size and style, if needed
set(gca, 'FontSize', 12);



%% Guardando las plots

%saveas(h2, ['c',num2str(controlador),'_v.png'])
%saveas(h3, ['c',num2str(controlador),'_w.png']) 
%saveas(h4, ['c',num2str(controlador),'_lr.png'])
saveas(h5, ['pso',num2str(controlador),'_pos.eps'],'epsc')







