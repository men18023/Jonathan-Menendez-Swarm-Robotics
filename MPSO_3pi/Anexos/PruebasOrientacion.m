% Initialize robotat 
clear;
close all;
load('Orientaciones.mat','bearing_deg');
robotat = robotat_connect();
 
% Define robot
r7 = robotat_3pi_connect(7);
robot7 = robotat_get_pose(robotat,7,'ZYX');

% Propiedades físicas del robot
MAX_WHEEL_VELOCITY = 800;
%WHEEL_RADIUS = 0.032;
WHEEL_RADIUS=32/2000;
MAX_SPEED = WHEEL_RADIUS * MAX_WHEEL_VELOCITY;
DISTANCE_FROM_CENTER=96/2000; 
WHEEL_RADIUS=32/2000;
goal1 = [0; 0];
% Posición
v0 = MAX_SPEED/12;
alpha = 0.9;
t = 0 ;
TIME_STEP = 32;
% PID Orientación
kpO = 1;
kiO = 0.1;
kdO = 0;
eO_D = 0;
eO_1 = 0;
EO = 0;
%%
while 1
    pos = robotat_get_pose(robotat,7,'eulxyz');
    pos(1:2);
    %xx = pos - pos2;
    theta = (pos(6)-bearing_deg(7))*pi/180;
    %theta = get_bearing_in_degrees(compass)
    
    e = [goal1(1)-pos(1);goal1(2)-pos(2)];
    thetag = atan2(e(2), e(1));
    
    eP = norm(e);
    eO = thetag - theta;
    eO = atan2(sin(eO), cos(eO));
    
    % %Control de velocidad lineal
    kP = v0 * (1-exp(-alpha*eP^2)) / eP;
    v = kP*eP
    %v = 0;
    
    % Control de velocidad angular
    eO_D = eO - eO_1;
    EO = EO + eO;
    w = kpO*eO + kiO*EO + kdO*eO_D
    eO_1 = eO;
    
    u = [v; w];
    
    lws = (v - w*DISTANCE_FROM_CENTER) / WHEEL_RADIUS;
    rws = (v + w*DISTANCE_FROM_CENTER) / WHEEL_RADIUS;
    if lws > 20
        lws = 20;
    end
    if lws < -20
        lws = -20;
    end
    if rws > 20
        rws = 20;
    end
    if rws < -20
        rws = -20;
    end
    disp(rws)
    disp(lws)
    robotat_3pi_set_wheel_velocities(r7, lws, rws);
    %wb_motor_set_velocity(left_wheel, lws);
    %wb_motor_set_velocity(right_wheel, rws);
    if goal1(1)-pos(1)<0.05 && goal1(2)-pos(2)<0.05; break; end
    % Se actualizan los tiempos actuales de simulación
    i = i + 1;
    t = t + TIME_STEP/1000;
end