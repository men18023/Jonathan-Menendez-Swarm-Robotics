% Define your original data
load('webots_test.mat')
x_original = webots_path(:,1);
y_original = webots_path(:,2);

% Desired number of points (including original points)
desired_points = 250;

% Initialize arrays to store interpolated points
x_interpolated = [];
y_interpolated = [];

% Interpolate between each pair of original points
for i = 1:numel(x_original) - 1
    x_interp = linspace(x_original(i), x_original(i + 1), desired_points / (numel(x_original) - 1));
    y_interp = linspace(y_original(i), y_original(i + 1), desired_points / (numel(y_original) - 1));
    x_interpolated = [x_interpolated, x_interp(1:end-1)];
    y_interpolated = [y_interpolated, y_interp(1:end-1)];
end

% Add the last point
x_interpolated = [x_interpolated, x_original(end)]';
y_interpolated = [y_interpolated, y_original(end)]';

% Plot the interpolated trajectory
%plot(x_interpolated, y_interpolated, 'b', 'LineWidth', 2);
%hold on;
%scatter(x_original, y_original, 'r', 'filled');
plot(tray(:,1),tray(:,2),'g')
legend('Interpolated Trajectory');
xlabel('X');
ylabel('Y');
title('Interpolated Trajectory');
grid on;
