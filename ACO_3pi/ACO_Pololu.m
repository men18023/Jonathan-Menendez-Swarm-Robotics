    % MATLAB controller for Webots
% File:          e-puck_MPSO_m.m
% Date:          06/20/2023
% Description:   e-puck_MPSO migration to matlab
% Author:        Jonathan Menéndez Cardona 18023
% Modifications:  

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:

robotat_disconnect(robotat);
%%
clear
close all;
clc;
% Load bearing angles for markers 1-15
load('Orientaciones.mat','bearing_deg','bearing_new');
load('pololu_best.mat');
%if ~exist('robotat', 'var')
robotat = robotat_connect();
%end
pause(0.5)
% Robotat Paramenters
Agent = 8;
%robotat = robotat_connect();
robot_agent = robotat_3pi_connect(Agent);
pause(2)
% Orientation offset for robots 1-10

%%
offset = zeros(10,1);
for b = 1:10
    bearing = abs(bearing_deg(b)) + 90;
    
    if (bearing > 180)
        bearing = bearing - 360;
    elseif (bearing < -180)
        bearing = bearing + 360;
    end
    offset(b) = bearing; 
end

%
robotat_3pi_set_wheel_velocities(robot_agent,-20,20);
% Pause while robotat connects (prevents errors on initial get_pose)
pause(2)
robotat_3pi_force_stop(robot_agent);
pause(0.2)
%iteration = 0;
%% Movimiento controlado
% Physical properties of robot (Pololu 3Pi+)
MAX_WHEEL_VELOCITY = 800;   % maximum rpm available for each wheel
WHEEL_RADIUS = 32/2000;     % radius in meters
MAX_SPEED = WHEEL_RADIUS * MAX_WHEEL_VELOCITY;
DISTANCE_FROM_CENTER = 96/2000; % distance from center to wheels in meters
%radio=WHEEL_RADIUS/2;
limiter = 70;     % controlled max rpm 
% Posición
v0 = MAX_SPEED/4; % initial speed
alpha = 0.8;      

% PID Orientación
kpO = 1;
kiO = 0.001;
kdO = 0;
eO_D = 0;
eO_1 = 0;
EO = 0;

% Generación de trayectoria por ruta entregada por ACO.m
% Trajectory generation for best path from ACO.m
pos_origin = robotat_get_pose(robotat,Agent,'eulxyz');

%interpolate_step = 0.005; % used on v1 trajectory generation
% v2 of trajectory generation
x_traj = [pos_origin(1); pololu_path(:, 1)]; 
y_traj = [pos_origin(2); pololu_path(:, 2)];
% Desired number of points (including origin points)
desired_points = 75 + 25 * (length(x_traj) - 2);

% Initialize arrays to store interpolated points
x_interpolated = [];
y_interpolated = [];

% Interpolate between each pair of original points
for i = 1:numel(x_traj) - 1
    x_interp = linspace(x_traj(i), x_traj(i + 1), desired_points / (numel(x_traj) - 1));
    y_interp = linspace(y_traj(i), y_traj(i + 1), desired_points / (numel(y_traj) - 1));
    x_interpolated = [x_interpolated, x_interp(1:end-1)];
    y_interpolated = [y_interpolated, y_interp(1:end-1)];
end

% Add the last point
x_interpolated = [x_interpolated'; x_traj(end)'];
y_interpolated = [y_interpolated'; y_traj(end)'];


%%xtray7 = (x_tray7(1):interpolate_step:x_tray7(end))';
% if x_tray(1) < x_tray(end)
%     xtray = (x_tray(1):interpolate_step:x_tray(end))';
%     ytray = interp1q(x_tray, y_tray, xtray);
% else
%     [unique_x_tray, idx] = unique(x_tray, 'stable');  % Remove duplicates and keep the original order
%     % Apply a modification to duplicates (e.g., add 0.1)
%     duplicate_indices = setdiff(1:length(x_tray), idx);
%     x_tray(duplicate_indices) = x_tray(duplicate_indices) - 0.01;
%     y_tray = y_tray(idx);  % Match y values to unique x values
% 
%     xtray = (x_tray(1):-interpolate_step:x_tray(end))';
%     x_tray = flipud(x_tray);
%     y_tray = flipud(y_tray);
%     ytray = interp1(x_tray, y_tray, xtray, 'linear', 'extrap'); % Ensure it extends to the last value
% end

%ytray7 = interp1q(x_tray7, y_tray7, xtray7);
%tray = [xtray,ytray];
traj = [x_interpolated,y_interpolated];

x = pos_origin(1); y = pos_origin(2);
goal = [x_traj(end), y_traj(end)];
lim = norm(goal-[x,y]);


% Initialization of history variables for controller_analysis.m
e_hist = zeros(2, length(traj)); 
trajectory = zeros(length(traj),2); 
v_hist = zeros(1, length(traj)); 
w_hist = zeros(1, length(traj)); 
rwheel_hist = zeros(1, length(traj)); 
lwheel_hist = zeros(1, length(traj)); 
phi_hist = zeros(2,length(traj));

% Ciclo decontrol
k=1;
rep = 0;
%block = 0;
pause(0.5)
%%
%while(k<length(tray))
pause(1)
while 1%(lim>0.04)     % break loop if position error is < 5cm 
    % break loop in case trajectory doesn't reach expected goal
    if k > length(traj) && rep <= 25
        k = length(traj);
        rep = rep + 1;
    end
    if  rep == 25  % repeats up to 25 times last point to reach ideal error
        break
    end
    lim = norm(goal-[x,y]);  % error from actual position to goal
    
    xi = robotat_get_pose(robotat,Agent,'eulxyz');
    x = xi(1); y = xi(2);  %theta = (xi(6)-90)*pi/180;
    theta = deg2rad(xi(6)+offset(Agent));
    xg = traj(k,1);
    yg = traj(k,2);
    e = [xg - x; yg - y];
    thetag = atan2(e(2), e(1));
    
    eP = norm(e);
    eO = angdiff(theta,thetag);
    ee = [eP;eO];
    
    % Lineal velocity control
    kP = v0 * (1-exp(-alpha*eP^2)) / eP;
    v = kP*eP;
    
    % Angular velocity control
    eO_D = eO - eO_1;
    EO = EO + eO;
    w = kpO*eO + kiO*EO + kdO*eO_D;
    eO_1 = eO;
    
    % Combination of controllers 
    u = [v; w];
    %u_hist = [u_hist, u];
    
    % Set wheel velocities in rad/s
    phi_L = (v - w * DISTANCE_FROM_CENTER) / WHEEL_RADIUS;
    phi_R = (v + w * DISTANCE_FROM_CENTER) / WHEEL_RADIUS;
    % Set wheel velocities in rpm
    phi_L = convangvel(phi_L, 'rad/s', 'rpm');  
    phi_R = convangvel(phi_R, 'rad/s', 'rpm');
    % Limit velocities of each wheel to avoid losing control
    if phi_L > limiter
        phi_L = limiter;
    end
    if phi_L < -limiter
        phi_L = -limiter;
    end
    if phi_R > limiter
        phi_R = limiter;
    end
    if phi_R < -limiter
        phi_R = -limiter;
    end

    % Save each iteration of the controller values on a history variable
    e_hist(:,k) = ee;
    trajectory(k,:) = [xi(1) xi(2)];
    v_hist(:,k) = v;
    w_hist(:,k) = w;
    rwheel_hist(:,k) = phi_R;
    lwheel_hist(:,k) = phi_L;
    goal = [x_traj(end), y_traj(end)];
    save('analysis.mat', 'trajectory', 'v_hist', 'w_hist', 'rwheel_hist', 'lwheel_hist', 'goal','-append')
    % Go to next desired point
    fprintf('%i\n', k);
    k=k+1;
    pause(0.01)
        % Send velocities for robot execution
%     if mod(k, 2) == 0
    robotat_3pi_set_wheel_velocities(robot_agent,phi_L,phi_R);
%     end
end
% Save last position for history variable and stop robot movement
pause(1);
if rep == 25 %% lim<0.04 ||
    xi = robotat_get_pose(robotat,Agent,'eulxyz');
    x = xi(1); y = xi(2);
    trajectory(k,:) = [xi(1), xi(2)];
    robotat_3pi_force_stop(robot_agent);
    pause(1)
    robotat_3pi_disconnect(robot_agent);
end
%robotat_disconnect(robotat);