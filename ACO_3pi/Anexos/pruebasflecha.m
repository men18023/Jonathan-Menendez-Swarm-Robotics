% Create a plot3 line
x = [0, 1]; % X-coordinates of the line
y = [0, 1]; % Y-coordinates of the line
z = [0, 1]; % Z-coordinates of the line

figure;
plot3(x, y, z, 'b', 'LineWidth', 2); % Plot the line
hold on;

% Add a label to the line
lineLabel = 'My Line'; % Your desired label

% Define the arrow properties (if needed)
% ...

% Plot the arrow (if needed)
% ...

hold off;

grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;
view(3); % Adjust the view as needed

% Add the legend entry for the line
legend(lineLabel);
