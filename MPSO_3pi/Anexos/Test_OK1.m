% MATLAB controller for Webots
% File:          e-puck_MPSO_m.m
% Date:          06/20/2023
% Description:   e-puck_MPSO migration to matlab
% Author:        Jonathan Menéndez Cardona 18023
% Modifications:  

% Cleans workspace and command window 
clear;
close all;
clc;
% Loads bearing angles 
load('Orientaciones.mat','bearing_deg','bearing_new');
% Adjust 3pi physical parameters
%MAX_WHEEL_VELOCITY = 800;
r = 0.016;
l = 0.048;
%MAX_SPEED = r * MAX_WHEEL_VELOCITY*2;
%ell=96/2000;
%radio=32/2000;
% Connects to robotat rerver and available agents
robotat = robotat_connect();
ang_seq = 'eulxyz';  % sequence for all get_pose 
first_agent = 7;     % first number of 3pi available
last_agent = 7;      % last number of 3pi available
Q_Agents = last_agent-first_agent+1;
for con = first_agent:last_agent
    eval(['robot_' num2str(con) '=robotat_3pi_connect(con);']);
end

TUC_CONTROLLER = 0;
PID_CONTROLLER = 1;
NKC_CONTROLLER_1 = 0;
NKC_CONTROLLER_2 = 0;
NKC_CONTROLLER_3 = 0;
LQR_CONTROLLER = 0;
LQI_CONTROLLER = 0;

HARDSTOP_FILTER = 0;

LQR_PID_COMBO = 0;

USE_BEARING = 0;

TIME_STEP = 32;
%MAX_SPEED = 6.28;
COMMUNICATION_CHANNEL = 1;
%ROBOT_RADIUS = 35.0;
%WHEEL_RADIUS = 20.5;

BENCHMARK_TYPE = 0;
TIME_DELTA = 0.032;

PSO_STEP = 1;

USE_STANDARD_PSO = 1;

INERTIA_TYPE = 1;

CONSTRICTION_FACTOR = 0.8;
COGNITIVE_WEIGTH = 2;
SOCIAL_WEIGTH = 10;

pause(2)

iteration = 0;


% Initialize all history and algoritm variables.
trajectory = cell(1, Q_Agents);
u_vw = cell(1, Q_Agents);
phi_L = cell(1, Q_Agents);
phi_R = cell(1, Q_Agents);
% Generate and store the arrays in the cell array
for i = 1:Q_Agents
    % Create a 2x1 vector filled with initial values
    vector = robotat_get_pose(robotat,i,ang_seq);
    vector0 = zeros(1,2);
    value = 0;
    % Store the vector in the cell array
    trajectory{i} = vector;
    u_vw{i} = vector;
    phi_L{i} = value;
    phi_L{i} = value;
end
%% Robot variables for algoritm
position_robot = zeros(Q_Agents,6);

% Posicion y velocidad nuevas del robot

old_velocity = zeros(Q_Agents,2);
new_velocity = zeros(Q_Agents,2);
theta_g = zeros(Q_Agents,1);

actual_position = zeros(Q_Agents,2);
old_position = zeros(Q_Agents,2);
new_position = zeros(Q_Agents,2);
theta_o = zeros(Q_Agents,1);
north = zeros(Q_Agents,1);
fitness_actual = zeros(Q_Agents,1);

best_local = zeros(Q_Agents,2);
fitness_local = zeros(Q_Agents,1);

best_global = [0,0];
fitness_global = zeros(Q_Agents,1);
fitness_global_ = 0;

epsilon = CONSTRICTION_FACTOR; % PSO constraint factor
c1 = COGNITIVE_WEIGTH; % PSO cognitive weight
c2 = SOCIAL_WEIGTH; % PSO social weight
%w = 0; % PSO inertia
rho1 = 0; % PSO cognitive uniformity
rho2 = 0; % PSO social uniformity
V_scaler = TIME_DELTA; % PSO velocity scaling factor

% Variables de calculo de Parametro PSO de Inercia
w_min = 0.5; % PSO inertia minimum
w_max = 1.0; % PSO inertia maximum
MAXiter = 10000; % Maximum number of iterations
iter = 0; % Iteration counter

% Variables de Proportional Integral Derivative controller (PID)
KP = K_PROPORTIONAL; % PID proportional gain
KI = K_INTEGRAL; % PID integral gain
KD = K_DERIVATIVE; % PID derivative gain
e_old = zeros(Q_Agents,1); % Previous PID error
E_old = zeros(Q_Agents,1); % Previous accumulated PID error
e_o = zeros(Q_Agents,1); % Current PID error
E_o = zeros(Q_Agents,1); % Current accumulated PID error
e_D = zeros(Q_Agents,1); % Differential error

% Variables de Non-lineal Kinematics controller (NKC)
alpha = zeros(Q_Agents,1); % Angle between robot-target vector and robot's front vector
beta = zeros(Q_Agents,1); % Target's orientation angle in inertial frame
rho_p = zeros(Q_Agents,1); % Euclidean distance between robot centroid and target
K_RHO = K_DISTANCE; % Distance control gain
K_A = K_alpha; % Orientation control gain
K_B = -K_beta; % Target rotation control gain

% Variables de Integrador Lineal Cuadratico (LQI)
delta_t = TIME_DELTA; % Sampling period
XI_X = zeros(Q_Agents,1); % X-axis integral error
XI_Y = zeros(Q_Agents,1); % Y-axis integral error

% Error de posicion entre robot y meta
e_x = zeros(Q_Agents,1); % Horizontal position difference
e_y = zeros(Q_Agents,1); % Vertical position difference
e_p = zeros(Q_Agents,1); % Euclidean distance between robot and target

% Variables de velocidades angulares pasadas de motores para Filtro de Picos
phi_r = zeros(Q_Agents,1); % Current right motor angular velocity
phi_l = zeros(Q_Agents,1); % Current left motor angular velocity
PhiR_old = zeros(Q_Agents,1); % Previous right motor angular velocity
PhiL_old = zeros(Q_Agents,1); % Previous left motor angular velocity

% Initialization of linear and angular velocity
v = zeros(Q_Agents,1);                     % Linear velocity of the robot
w = zeros(Q_Agents,1);                     % Angular velocity of the robot
orientation_angle = zeros(Q_Agents,1);
u_1 = zeros(Q_Agents,1);
u_2 = zeros(Q_Agents,1);

v_hist = [];
w_hist = [];
phi_l_hist = [];
phi_r_hist = [];
number_iteration = 0;
index1 = 0;
state = false;
%% Movimiento controlado

goal_1 = robotat_get_pose(robotat,1,ang_seq);
goal = [1,1];
for gen = first_agent:last_agent
    numb = gen-first_agent+1;
    %eval(['pos_origin' num2str(gen) '=robotat_get_pose(robotat,con,ang_seq);']);
    pos_origin = robotat_get_pose(robotat,gen,ang_seq);
    pos_origin_temp = pos_origin(1:2);
    eval(['tray_x' num2str(gen) '=linspace(pos_origin_temp(1), goal(1), 200);']);
    %tray_x7 = linspace(pos_origin(1), goal(1), 200);
    eval(['tray_y' num2str(gen) '=linspace(pos_origin_temp(2), goal(2), 200);']);
    x_temp = eval(['tray_x' num2str(gen)]);
    y_temp = eval(['tray_y' num2str(gen)]);
    %tray_y7 = linspace(pos_origin(2), goal(2), 200);
    eval(['tray{' num2str(numb) '}' '=[transpose(x_temp),transpose(y_temp)];']);
end
% Ciclo decontrol

k=1;
%%
while(k<length(tray{1}))
    %tStart = tic; 
    disp(k)
    for agent_vel = first_agent:last_agent
           numb = agent_vel-first_agent+1;
           [phi,xi,yi,u] = PID_controller1(robotat,agent_vel,bearing_new(agent_vel),tray{numb},k);
           eval(['robotat_3pi_set_wheel_velocities(robot_' num2str(agent_vel) ',phi(1),phi(2))']);
           %display(phi)
           %eval(['phi_L{' num2str(numb) '}' '=[phi_L{' num2str(numb) '}; phi(1)']);
           %eval(['phi_R{' num2str(numb) '}' '=[phi_R{' num2str(numb) '}; phi(2)']);
           %eval(['trajectory{' num2str(numb) '}' '=[trajectory{' num2str(numb) '}; xi, yi];']);
           %eval(['u_vw{' num2str(numb) '}' '=[trajectory{' num2str(numb) '}; u];']);
           %trajectory = [trajectory; [xi(1), xi(2)]];
    end
%     trajectory1 = [trajectory1; [xi(1), xi(2)]];
%     v_hist = [v_hist; v];
%     w_hist = [w_hist; w];
%     rwheel_hist = [rwheel_hist; phi_R];
%     lwheel_hist = [lwheel_hist; phi_L];
%     goal = [x_tray7(end), y_tray7(end)];
%     save('analysis.mat', 'trajectory', 'v_hist', 'w_hist', 'rwheel_hist', 'lwheel_hist', 'goal','-append')
    k=k+1;
end

parfor ctrl = k:length(tray{1})
    
end    
%%
%tEnd = toc(tStart);      % pair 2: toc
pause(1);
for disc = first_agent:last_agent
    %sequence = 'eulxyz';
    eval(['robotat_3pi_force_stop(robot_' num2str(disc) ');']);
    eval(['robotat_3pi_disconnect(robot_' num2str(disc) ');']);
end