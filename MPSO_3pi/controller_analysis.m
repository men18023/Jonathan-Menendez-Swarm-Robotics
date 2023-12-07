% Código basado en el de Aldo, pues se quería comparar sus gráficas del PSO
% con las gráficas del ACO.
close all
filename = 'analysis_5_4';
load(strcat(filename,'.mat'))
%load('analysis.mat')
%t = 32*linspace(0, length(v_hist)-1, length(v_hist));
controlador = 31;
%%
h2 = figure(1);
ID = 1;
str = 'Velocidad lineal';
set(h2, 'units', 'points', 'position', [60, 95, 620, 420], 'name', str);
set(h2, 'color', 'w');

hold on;

line_styles = {'-','--',':','-.'}; % Define different line styles for each agent
colors = lines(last_agent - first_agent + 1); % Use a different set of colors for better distinguishability

for agent = first_agent:last_agent
    col = agent - first_agent + 1;
    plot(1:length(v_hist), v_hist(:, agent), 'LineWidth', 2, 'LineStyle', line_styles{mod(col,4)+1}, 'Color', colors(col,:));
end

hold off;

xlim([0, size(v_hist, 1)]);
grid on; grid minor;
xlabel('Iteración', 'FontSize', 16); ylabel('Velocidad (m/s)', 'FontSize', 16);

% Add a legend with labels for the trajectories
legend_text = cell(last_agent - first_agent + 1, 1);
for agent = first_agent:last_agent
    agent_index = agent - first_agent + 1;
    legend_text{agent_index} = sprintf('Agente %d', agent);
end

legend(legend_text, 'Interpreter', 'tex', 'Location', 'Best');

% Customize the appearance
%title('Velocity History', 'FontSize', 18, 'FontWeight', 'bold');
set(gca, 'FontSize', 12);
set(gca, 'LineWidth', 1.2);

% Add grid lines for better readability
grid on; grid minor;

% Adjust legend font size
legend_font_size = 12;
legend('FontSize', legend_font_size);

% You can also adjust the font size and style, if needed
set(gca, 'FontSize', 12);


%%
h3 = figure(2);
str = 'Velocidad Angular';
set(h3, 'units', 'points', 'position', [60,95,620,420], 'name', str);
set(h3, 'color', 'w');
hold on;

line_styles = {'-','--',':','-.'}; % Define different line styles for each agent
colors = lines(last_agent - first_agent + 1); % Use a different set of colors for better distinguishability

for agent = first_agent:last_agent
    col = agent - first_agent + 1;
    plot(1:length(w_hist), w_hist(:, agent), 'LineWidth', 2, 'LineStyle', line_styles{mod(col,4)+1}, 'Color', colors(col,:));
end

hold off;

xlim([0, size(w_hist, 1)]);
grid on; grid minor;
xlabel('Iteración','FontSize',16); ylabel('Velocidad (rad/s)','FontSize',16);

% Add a legend with labels for the trajectories
legend_text = cell(last_agent - first_agent + 1, 1);
for agent = first_agent:last_agent
    agent_index = agent - first_agent + 1;
    legend_text{agent_index} = sprintf('Agente %d', agent);
end

legend(legend_text, 'Interpreter', 'tex', 'Location', 'Best');

% Customize the appearance
%title('Angular Velocity History', 'FontSize', 18, 'FontWeight', 'bold');
set(gca, 'FontSize', 12);
set(gca, 'LineWidth', 1.2);

% Add grid lines for better readability
grid on; grid minor;

% Adjust legend font size
legend_font_size = 12;
legend('FontSize', legend_font_size);

%%
% Assuming the previous code has been executed and you have h4, rwheel_hist, lwheel_hist, first_agent, last_agent, and wheel_colors available

% Assuming the previous code has been executed and you have h4, rwheel_hist, lwheel_hist, first_agent, last_agent, and wheel_colors available

agent_range = first_agent:last_agent;
num_agents = last_agent - first_agent + 1;
agents_per_subplot = 2;
num_subplots = ceil(num_agents / agents_per_subplot);

for subplot_index = 1:num_subplots
    start_agent = (subplot_index - 1) * agents_per_subplot + 1;
    end_agent = min(subplot_index * agents_per_subplot, num_agents);

    h = figure(2 + subplot_index); % Use a different figure number for each subplot
    str = sprintf('Velocidad en las ruedas - Agents %d to %d', start_agent, end_agent);
    set(h, 'units', 'points', 'position', [60, 95, 620, 420], 'name', str);
    set(h, 'color', 'w');
    ax = axes('Parent', h, 'Position', [0.0623716632443532 0.105031948881789 0.907765757474463 0.870952303086714]);
    hold(ax, 'on');

    for col = agent_range(start_agent:end_agent)
        plot(1:length(rwheel_hist), rwheel_hist(:, col), 'LineWidth', 2, 'LineStyle', '-', 'Color', wheel_colors(col - first_agent + 1, :));
    end

    for col = agent_range(start_agent:end_agent)
        plot(1:length(lwheel_hist), lwheel_hist(:, col), 'LineWidth', 2, 'LineStyle', '--', 'Color', wheel_colors(col - first_agent + 1, :));
    end

    plot([0 length(lwheel_hist)], [50 50], 'LineStyle', '--', 'Color', 'k');
    plot([0 length(lwheel_hist)], [-50 -50], 'LineStyle', '--', 'Color', 'k');

    hold off;

    xlim([0, length(lwheel_hist)]);
    ylim([-70, 70]);
    grid on; grid minor;
    xlabel('Iteración', 'FontSize', 16); ylabel('Velocity (rpm)', 'FontSize', 16);

    % Adjust legend to show entries for each column
    legend_text = cell(2 * (end_agent - start_agent + 1) + 2, 1);
    legend_text_counter = 1;

    for col = agent_range(start_agent:end_agent)
        legend_text{legend_text_counter} = sprintf('Velocidad motor derecho - Agente %d', col);
        legend_text_counter = legend_text_counter + 1;
    end

    for col = agent_range(start_agent:end_agent)
        legend_text{legend_text_counter} = sprintf('Velocidad motor izquierdo - Agente %d', col);
        legend_text_counter = legend_text_counter + 1;
    end

    legend_text{end-1} = '50 rpm threshold';
    legend_text{end} = '-50 rpm threshold';

    legend('off'); % Turn off the original legend

    % Create a new legend with adjusted properties
    lgd = legend(ax, legend_text, 'Location', 'southwest', 'FontSize', 10);
    lgd.Position = [0.2, 0.12, 0.2, 0.2]; % Adjust the position and size as needed

    box(ax, 'on');
    grid(ax, 'on');
    set(ax, 'XMinorGrid', 'on', 'YMinorGrid', 'on', 'ZMinorGrid', 'on');

    % You can also add a title if needed
    % title('Angular Velocities History');

    % You can also adjust the font size and style, if needed
    set(gca, 'FontSize', 12);
    set(gca, 'LineWidth', 1);
    set(gca, 'FontSize', 12);
end


%%
% Create a figure
h5 = figure(6);
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

% Adjust legend font size

% Add a title
% title('Trajectories of Agents');

% You can also adjust the font size and style, if needed
set(gca, 'FontSize', 12);
set(gca, 'LineWidth', 1);
set(gca, 'FontSize', 12);


%% Guardando las plots

saveas(h2, [filename,'_v.png'])
saveas(h3, [filename,'_w.png']) 
saveas(figure(3), [filename,'_lr1.png'])
saveas(figure(4), [filename,'_lr2.png'])
saveas(figure(5), [filename,'_lr3.png'])
saveas(h5, [filename,'_pos.png'])

saveas(h2, [filename,'_v.eps'],'epsc')
saveas(h3, [filename,'_w.eps'],'epsc') 
saveas(figure(3), [filename,'_lr1.eps'],'epsc')
saveas(figure(4), [filename,'_lr2.eps'],'epsc')
saveas(figure(5), [filename,'_lr3.eps'],'epsc')
saveas(h5, [filename,'_pos.eps'],'epsc')








