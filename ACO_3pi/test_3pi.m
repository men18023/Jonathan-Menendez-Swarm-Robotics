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
load('webots_test.mat');
% Adjust 3pi physical parameters
%MAX_WHEEL_VELOCITY = 800;
r = 0.016;
l = 0.048;
%MAX_SPEED = r * MAX_WHEEL_VELOCITY*2;
%ell=96/2000;
%radio=32/2000;
% Robotat Paramenters
robotat = robotat_connect();
robot7 = robotat_3pi_connect(7);
%robot8 = robotat_3pi_connect(8);
%robot9 = robotat_3pi_connect(9);
% Define the goal position
%goal_x = 0;
%goal_y = 0;

% Define control gains
Kp_linear = 0.5;
Kp_angular = 0.05;

e_hist = [0;0];
trajectory = [];
v_hist = [];
w_hist = [];
rwheel_hist = [];
lwheel_hist = [];
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
v0 = MAX_SPEED/8;
alpha = 0.9;

% PID Orientación
kpO = 1;
kiO = 0.001;
kdO = 0;
eO_D = 0;
eO_1 = 0;
EO = 0;
pos_origin = robotat_get_pose(robotat,7,'eulxyz');
goal = [0,0];
interpolate_step = 0.005;
x_tray7 = [pos_origin(1); webots_path(:, 1)]; 
y_tray7 = [pos_origin(2); webots_path(:, 2)];
xtray7 = (x_tray7(1):interpolate_step:x_tray7(end))';
ytray7 = interp1q(x_tray7, y_tray7, xtray7);
tray7 = [xtray7,ytray7];
% Ciclo decontrol
k=1;
while(k<length(tray7))
    xi = robotat_get_pose(robotat,7,'XYZ');
    x = xi(1); y = xi(2);  theta = (xi(6)-90)*pi/180;
    
    xg = tray7(k,1);
    yg = tray7(k,2);
    e = [xg - x; yg - y];
    thetag = atan2(e(2), e(1));
    
    eP = norm(e);
    eO = angdiff(theta,thetag);
    ee = [eP;eO];
    e_hist = [e_hist, ee];
    % Control de velocidad lineal
    kP = v0 * (1-exp(-alpha*eP^2)) / eP;
    v = kP*eP;
    
    % Control de velocidad angular
    eO_D = eO - eO_1;
    EO = EO + eO;
    w = kpO*eO + kiO*EO + kdO*eO_D;
    eO_1 = eO;
    
    % Se combinan los controladores
    u = [v; w];
    %u_hist = [u_hist, u];
    phi_R = (v+w*ell)/radio;
    phi_L = (v-w*ell)/radio;
    phi_L = convangvel(phi_L, 'rad/s', 'rpm');
    phi_R = convangvel(phi_R, 'rad/s', 'rpm');
    if phi_L > 75
        phi_L = 75;
    end
    if phi_L < -75
        phi_L = -75;
    end
    if phi_R > 75
        phi_R = 75;
    end
    if phi_R < -75
        phi_R = -75;
    end
    robotat_3pi_set_wheel_velocities(robot7,phi_L,phi_R);
    trajectory = [trajectory; [xi(1), xi(2)]];
    v_hist = [v_hist; v];
    w_hist = [w_hist; w];
    rwheel_hist = [rwheel_hist; phi_R];
    lwheel_hist = [lwheel_hist; phi_L];
    goal = [x_tray7(end), y_tray7(end)];
    save('analysis.mat', 'trajectory', 'v_hist', 'w_hist', 'rwheel_hist', 'lwheel_hist', 'goal','-append')
    k=k+1
end
pause(2);
robotat_3pi_force_stop(robot7);
robotat_3pi_disconnect(robot7);