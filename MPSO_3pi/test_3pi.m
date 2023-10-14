% MATLAB controller for Webots
% File:          e-puck_MPSO_m.m
% Date:          06/20/2023
% Description:   e-puck_MPSO migration to matlab
% Author:        Jonathan Menéndez Cardona 18023
% Modifications:  

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
clear;
close all;
clc;
% Load bearing angles for markers 1-15
load('Orientaciones.mat','bearing_deg');
% Adjust 3pi physical parameters
%MAX_WHEEL_VELOCITY = 800;
r = 0.016;
l = 0.048;
%MAX_SPEED = r * MAX_WHEEL_VELOCITY*2;
%ell=96/2000;
%radio=32/2000;
% Robotat Paramenters
%robotat = robotat_connect();
%robot7 = robotat_3pi_connect(7);
%robot8 = robotat_3pi_connect(8);
%robot9 = robotat_3pi_connect(9);
% Define the goal position
goal_x = 0;
goal_y = 0;

% Define control gains
Kp_linear = 0.5;
Kp_angular = 0.05;

pause(2)
iteration = 0;
%% Movimiento controlado
% Propiedades físicas del robot
MAX_WHEEL_VELOCITY = 800;
WHEEL_RADIUS = 0.032;
MAX_SPEED = WHEEL_RADIUS * MAX_WHEEL_VELOCITY;
ell=96/2000; 
radio=32/2000;

% Posición
v0 = MAX_SPEED/12;
alpha = 0.9;

% PID Orientación
kpO = 1;
kiO = 0.001;
kdO = 0;
eO_D8 = 0;
eO_18 = 0;
EO8 = 0;
eO_D7 = 0;
eO_17 = 0;
EO7 = 0;
pos_goal = robotat_get_pose(robotat,9,'eulxyz');
pos_origin8 = robotat_get_pose(robotat,8,'eulxyz');
pos_origin7 = robotat_get_pose(robotat,7,'eulxyz');
goal = [pos_goal(1)-0.1,pos_goal(2)-0.1];
tray_x8 = linspace(pos_origin8(1), goal(1), 100);
tray_y8 = linspace(pos_origin8(2), goal(2), 100);
tray8 = [tray_x8',tray_y8'];
tray_x7 = linspace(pos_origin7(1), goal(1), 100);
tray_y7 = linspace(pos_origin7(2), goal(2), 100);
tray7 = [tray_x7',tray_y7'];
% Ciclo decontrol
k=1;
bearing = bearing_deg;
for adj = 1:length(bearing)
    if bearing(adj) < 0
        bearing(adj) =  bearing(adj) + 360;
    end
end
%%
while(k<length(tray7))
    xi8 = robotat_get_pose(robotat,8,'XYZ');
    xi7 = robotat_get_pose(robotat,7,'XYZ');
    x8 = xi8(1); y8 = xi8(2);  theta8 = (xi8(6)-bearing(8))*pi/180;
    x7 = xi7(1); y7 = xi7(2);  theta7 = (xi7(6)-bearing(7))*pi/180;
    xg8 = tray8(k,1);
    yg8 = tray8(k,2);
    xg7 = tray7(k,1);
    yg7 = tray7(k,2);
    e8 = [xg8 - x8; yg8 - y8];
    e7 = [xg7 - x7; yg7 - y7];
    thetag8 = atan2(e8(2), e8(1));
    thetag7 = atan2(e7(2), e7(1));
    eP8 = norm(e8);
    eP7 = norm(e7);
    eO8 = angdiff(theta8,thetag8);
    eO7 = angdiff(theta7,thetag7);
    
    % Control de velocidad lineal
    kP8 = v0 * (1-exp(-alpha*eP8^2)) / eP8;
    v8 = kP8*eP8;
    kP7 = v0 * (1-exp(-alpha*eP7^2)) / eP7;
    v7 = kP7*eP7;
    % Control de velocidad angular
    eO_D8 = eO8 - eO_18;
    EO8 = EO8 + eO8;
    w8 = kpO*eO8 + kiO*EO8 + kdO*eO_D8;
    eO_18 = eO8;
    eO_D7 = eO7 - eO_17;
    EO7 = EO7 + eO7;
    w7 = kpO*eO7 + kiO*EO7 + kdO*eO_D7;
    eO_17 = eO7;
    % Se combinan los controladores
    u8 = [v8; w8];
    u7 = [v7; w7];
    phi_R8 = (v8+w8*ell)/radio;
    phi_L8 = (v8-w8*ell)/radio;
    phi_L8 = convangvel(phi_L8, 'rad/s', 'rpm');
    phi_R8 = convangvel(phi_R8, 'rad/s', 'rpm');
    phi_R7 = (v7+w7*ell)/radio;
    phi_L7 = (v7-w7*ell)/radio;
    phi_L7 = convangvel(phi_L7, 'rad/s', 'rpm');
    phi_R7 = convangvel(phi_R7, 'rad/s', 'rpm');
    if phi_L8 > 50
        phi_L8 = 50;
    end
    if phi_L8 < -50
        phi_L8 = -50;
    end
    if phi_R8 > 50
        phi_R8 = 50;
    end
    if phi_R8 < -50
        phi_R8 = -50;
    end
    if phi_L7 > 50
        phi_L7 = 50;
    end
    if phi_L7 < -50
        phi_L7 = -50;
    end
    if phi_R7 > 50
        phi_R7 = 50;
    end
    if phi_R7 < -50
        phi_R7 = -50;
    end
    %robotat_3pi_set_wheel_velocities(robot8,phi_L8,phi_R8);
    robotat_3pi_set_wheel_velocities(robot7,phi_L7,phi_R7);
    k=k+1
end
%%
pause(2);
robotat_3pi_force_stop(robot7);
robotat_3pi_disconnect(robot7);
robotat_3pi_force_stop(robot8);
robotat_3pi_disconnect(robot8);