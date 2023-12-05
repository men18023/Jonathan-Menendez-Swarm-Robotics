% Código basado en el de Aldo, pues se quería comparar sus gráficas del PSO
% con las gráficas del ACO.
load('analysis_1_2.mat')
%load('analysis.mat')
%t = 32*linspace(0, length(v_hist)-1, length(v_hist));
controlador = 31;
h2 = figure(1);
ID = 1;
str = 'Velocidad lineal';
set(h2, 'units', 'points', 'position', [60, 95, 620, 420], 'name', str);
set(h2, 'color', 'w');

hold on;

for col = first_agent:last_agent
    plot(1:length(v_hist), v_hist(:, col), 'LineWidth', 2);
end

hold off;

xlim([0, size(v_hist, 1)]);
grid on; grid minor;
xlabel('Iteration', 'FontSize', 16); ylabel('Velocidad (rpm)', 'FontSize', 16);

% Add a legend with labels for the trajectories
legend_text = cell(last_agent - first_agent + 1, 1);
for agent = first_agent:last_agent
    agent_index = agent - first_agent + 1;
    legend_text{agent_index} = sprintf('Agente %d', agent);
end
% legend_text{end} = 'Global Best';

legend(legend_text, 'Interpreter', 'tex', 'Location', 'Best');

% You can also add a title if needed
% title('Velocity History');

% You can also adjust the font size and style, if needed
set(gca, 'FontSize', 12);



% 
%% Velocidad angular de Robot ------------------
h3 = figure(2);
str = 'Velocidad lineal';
set(h3, 'units', 'points', 'position', [60,95,620,420], 'name', str);
set(h3, 'color', 'w');
hold on;

for col = first_agent:last_agent
    plot(1:length(w_hist), w_hist(:, col), 'LineWidth', 2);
end

xlim([0, size(w_hist, 1)]);
grid on; grid minor;
xlabel('Iteration','FontSize',16); ylabel('Velocidad (rpm)','FontSize',16);
legend_text = cell(last_agent - first_agent + 1, 1);
for agent = first_agent:last_agent
    agent_index = agent - first_agent + 1;
    legend_text{agent_index} = sprintf('Agente %d', agent);
end
legend(legend_text, 'Interpreter', 'tex', 'Location', 'Best');

%legend_text{end} = 'Global Best';
%lgd.FontSize = 14;
%%
h4 = figure(3);
str = 'Velocidad lineal';
set(h4, 'units', 'points', 'position', [60, 95, 620, 420], 'name', str);
set(h4, 'color', 'w');
ax = axes('Parent', h4, 'Position', [0.0623716632443532 0.105031948881789 0.907765757474463 0.870952303086714]);
hold(ax, 'on');

% Define custom colors for the wheels
wheel_colors = lines(last_agent - first_agent + 1);

for col = first_agent:last_agent
    plot(1:length(rwheel_hist), rwheel_hist(:, col), 'LineWidth', 2, 'LineStyle', '-', 'Color', wheel_colors(col - first_agent + 1, :));
end

for col = first_agent:last_agent
    plot(1:length(lwheel_hist), lwheel_hist(:, col), 'LineWidth', 2, 'LineStyle', '--', 'Color', wheel_colors(col - first_agent + 1, :));
end

plot([0 length(lwheel_hist)], [50 50], 'LineStyle', '--', 'Color', 'k');
plot([0 length(lwheel_hist)], [-50 -50], 'LineStyle', '--', 'Color', 'k');

hold off;

xlim([0, length(lwheel_hist)]);
ylim([-70, 70]);
grid on; grid minor;
xlabel('Time (ms)', 'FontSize', 16); ylabel('Velocity (rad/s)', 'FontSize', 16);

% Adjust legend to show entries for each column
legend_text = cell(2 * (last_agent - first_agent + 1) + 2, 1);
legend_text_counter = 1;

for col = first_agent:last_agent
    legend_text{legend_text_counter} = sprintf('Velocidad motor derecho - Agente %d', col);
    legend_text_counter = legend_text_counter + 1;
end

for col = first_agent:last_agent
    legend_text{legend_text_counter} = sprintf('Velocidad motor izquierdo - Agente %d', col);
    legend_text_counter = legend_text_counter + 1;
end

legend_text{end-1} = '50 rpm threshold';
legend_text{end} = '-50 rpm threshold';

legend(legend_text, 'Location', 'best', 'FontSize', 14);

box(ax, 'on');
grid(ax, 'on');
set(ax, 'XMinorGrid', 'on', 'YMinorGrid', 'on', 'ZMinorGrid', 'on');

% You can also add a title if needed
% title('Angular Velocities History');

% You can also adjust the font size and style, if needed
set(gca, 'FontSize', 12);


% set(lgd,'Position',[0.0672128188013608 0.112952606873517 0.366543347512945 0.104832270655769],'FontSize',14);
%%
% Create a figure
h5 = figure(4);
str = 'Velocidad lineal';
set(h5, 'units', 'points', 'position', [60, 95, 520, 420], 'name', str);

% Define custom colors for the trajectories
trajectory_colors = lines(last_agent - first_agent + 1);

% Plot trajectories with custom line styles and colors
for agent = first_agent:last_agent
    agent_index = agent - first_agent + 1;
    plot(trajectory_hist(:, agent), trajectory_hist(:, agent + Q_Agents+1), 'LineWidth', 2, 'Color', trajectory_colors(agent_index, :), 'LineStyle', '-');
    hold on;
end

% Customize the view
view(0, 90);

% Plot the goal as a filled black circle
scatter(best_global(1), best_global(2), 500, 'k', 'filled');

% Set axis limits and grid
xlim([-2 2]);
ylim([-2.5 2.5]);
grid on;
grid minor;

% Set DataAspectRatio and make axes equal
daspect([1 1.2 1]);

% Label the axes
xlabel('X (m)');
ylabel('Y (m)');

% Get the axis limits
xLimits = xlim;
yLimits = ylim;

% Set figure position to fit the graph
figurePosition = [100, 100, diff(xLimits), diff(yLimits)] + [0, 0, 500, 500]; % Adjust the last two values for padding
set(h5, 'Position', figurePosition);

% Add a legend with labels for the trajectories
legend_text = cell(last_agent - first_agent + 2, 1);
for agent = first_agent:last_agent
    agent_index = agent - first_agent + 1;
    legend_text{agent_index} = sprintf('Agente %d', agent);
end
legend_text{end} = 'Global Best';

legend(legend_text, 'Interpreter', 'tex', 'Location', 'Best');

% Set the background color of the figure to white
set(h5, 'color', 'w');

% Add a title
% title('Trajectories of Agents');

% You can also adjust the font size and style, if needed
set(gca, 'FontSize', 12);


%% Guardando las plots

%saveas(h2, ['c',num2str(controlador),'_v.png'])
%saveas(h3, ['c',num2str(controlador),'_w.png']) 
%saveas(h4, ['c',num2str(controlador),'_lr.png'])
%saveas(h5, ['pso',num2str(controlador),'_pos.eps'],'epsc')








