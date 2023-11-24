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
robotat = robotat_connect();
%robot7 = robotat_3pi_connect(8);
%robot9 = robotat_3pi_connect(9);
first_agent = 7;  % first number of 3pi available
last_agent = 8;   % last number of 3pi available
Q_Agents = last_agent-first_agent+1;
for con = first_agent:last_agent
    eval(['robot_' num2str(con) '=robotat_3pi_connect(con);']);
end
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
ang_seq = 'eulxyz';
 goal = [0,0];
for gen = first_agent:last_agent
    numb = gen-first_agent+1;
    %eval(['pos_origin' num2str(gen) '=robotat_get_pose(robotat,con,ang_seq);']);
    pos_origin = robotat_get_pose(robotat,first_agent:last_agent,ang_seq);
    pos_origin_temp = pos_origin(numb);
    eval(['tray_x' num2str(gen) '=linspace(pos_origin_temp(1), goal(1), 200);']);
    %tray_x7 = linspace(pos_origin7(1), goal(1), 200);
    eval(['tray_y' num2str(gen) '=linspace(pos_origin_temp(1), goal(2), 200);']);
    x_temp = eval(['tray_x' num2str(gen)]);
    y_temp = eval(['tray_y' num2str(gen)]);
    %tray_y7 = linspace(pos_origin7(2), goal(2), 200);
    eval(['tray' num2str(gen) '=[transpose(x_temp),transpose(y_temp)];']);
    %tray7 = [tray_x7',tray_y7'];
end
% Ciclo decontrol
k=1;
%%
while(k<length(tray7))
    k0 = '---'
    xi = robotat_get_pose(robotat,first_agent:last_agent,'eulxyz');

    x = xi(:,1); y = xi(:,2);  theta = (xi(:,6)-11)*pi/180;
%     rad = atan2(xi7(2),xi7(1))-deg2rad(bearing_deg(8));
%     if rad < 0
%         rad = 2*pi + rad;
%     end
%     theta7 = deg2rad(270)-rad;
%     if theta7 < 0
%         theta7 = 2*pi + theta7;
%     end
%     disp(theta7);
    %marker 7 = -90
    %marker 8 = -198
    %marker 4 = -165
    for g = first_agent:last_agent
        numb = g-first_agent+1;
        eval(['xg' num2str(g) '=tray' num2str(g) '(1,1)']);
        eval(['yg' num2str(g) '=tray' num2str(g) '(1,2)']);
        %xg = tray7(k,1);
        %yg = tray7(k,2);
        eval(['e' num2str(g) '=[xg' num2str(g) '- x(numb);'...
            'yg' num2str(g) '- y(numb)' ' ]']);
        thetag(numb) = eval(['atan2(e' num2str(g) '(2),e' num2str(g) '(1))']);
        eP(numb) = eval(['norm(e' num2str(g) ')']);
    end
    %e7 = [xg7 - x7; yg7 - y7];

    %thetag = atan2(e7(2), e7(1));
    %disp(thetag7)
    %eP7 = norm(e7);

    eO7 = angdiff(theta7,thetag7);
    disp(eO7)
    % Control de velocidad lineal

    kP7 = v0 * (1-exp(-alpha*eP7^2)) / eP7;
    v7 = kP7*eP7;
    % Control de velocidad angular

    eO_D7 = eO7 - eO_17;
    EO7 = EO7 + eO7;
    w7 = kpO*eO7 + kiO*EO7 + kdO*eO_D7;
    eO_17 = eO7;
    % Se combinan los controladores
    u7 = [v7; w7];
    
    phi_R7 = (v7+w7*ell)/radio;
    phi_L7 = (v7-w7*ell)/radio;
    phi_L7 = convangvel(phi_L7, 'rad/s', 'rpm');
    phi_R7 = convangvel(phi_R7, 'rad/s', 'rpm');
    
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
    %robotat_3pi_set_wheel_velocities(robot7,phi_L7,phi_R7);
    k=k+1;
end
%%
pause(1);
robotat_3pi_force_stop(robot7);
robotat_3pi_disconnect(robot7);