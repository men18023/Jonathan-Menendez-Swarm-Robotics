% MATLAB controller for Webots
% File:          e-puck_MPSO_m.m
% Date:          06/20/2023
% Description:   e-puck_MPSO migration to matlab
% Author:        rob_fit_actonathan Menéndez Cardona 18023
% Modifications:  

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;
clear;
close all;
clc;
%wb_console_print(probando, WB_STDOUT)
load('Orientaciones.mat','bearing_deg','bearing_new');

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

% Propiedades físicas del robot
MAX_WHEEL_VELOCITY = 800;
WHEEL_RADIUS = 0.016;
ROBOT_RADIUS = 0.048;
MAX_SPEED = WHEEL_RADIUS * MAX_WHEEL_VELOCITY*2;
v0 = MAX_SPEED/12;
alpha_v = 0.9;
%ell=96/2000; 
%radio=32/2000;

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

%PID Parameters 0.5 0.1 0.001 (I = 0.25 con LQR para simular PSO-TP)
K_PROPORTIONAL = 1;
K_INTEGRAL = 0.001;
K_DERIVATIVE = 0;

%NKC Parameters (K_p > 0; K_b < 0; K_a > K_p) Kp = 0.01
K_DISTANCE = 0.1; 
K_alpha = 0.5;
K_beta = 0.05;

MAX_CHANGE = 1.00;

%% Robotat Configuration
robotat = robotat_connect();
pause(1);
%pos1 = robotat_get_pose(robotat,1,'eulxyz');

agents = cell(1,10);
first_agent = 2;  % first number of 3pi available
last_agent = 3;   % last number of 3pi available
Q_Agents = last_agent-first_agent+1;
for con = first_agent:last_agent
    eval(['robot_' num2str(con) '=robotat_3pi_connect(con);']);
end
%robotat_disconnect(robotat);
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
pause(2)

%% Loop
while abs(mean(e_p(vel))) > 0.5
    number_iteration = number_iteration + 1;
    disp(number_iteration);
    %position_robot((1:Q_Agents),:) = robotat_get_pose(robotat,(1:Q_Agents),'eulaxyz')
    
    if state == false
    position_robot((1:Q_Agents),:) = robotat_get_pose(robotat,(first_agent:last_agent),'eulzyx');
    actual_position(1:Q_Agents,1) = position_robot(1:Q_Agents,1);   % Coordenada en erob_fit_acte Z de simulacion es coordenada X
    actual_position(1:Q_Agents,2) = position_robot(1:Q_Agents,2);   % Coordenada en erob_fit_acte X de simulacion es coordenada Y
    best_local(1:Q_Agents,:) = actual_position(1:Q_Agents,:);
    best_global(1:Q_Agents,:) = actual_position(1:Q_Agents,:);
   
    % Calcular Fitness Value
    for i = 1:Q_Agents
        fitness_local(i) = fitness(actual_position(i,1), actual_position(i,2), BENCHMARK_TYPE);
        fitness_global(i) = fitness(actual_position(i,1), actual_position(i,2), BENCHMARK_TYPE);
    end
    fitness_global_ = min(fitness_global);
    state = true;
    end
%     if state == true
%         break
%     end
%%%%%%%%%%%%%%%%%
    position_robot((1:Q_Agents),:) = robotat_get_pose(robotat,(first_agent:last_agent),'eulaxyz');
    for bea = 1:Q_Agents   
        theta_o = (position_robot(1:Q_Agents,6)-bearing_new(first_agent:last_agent))*pi/180;
    end
% ------------------------- ACTUALIZACION DE LOCAL AND GLOBAL BESTS -------------------------    
% 
    % Calcular Fitness Value de posicion actiaul
    actual_position = position_robot;
    %actual_position = position_robot;
    for rob_fit_act = 1:Q_Agents
    fitness_actual(rob_fit_act) = fitness(actual_position(rob_fit_act,1), actual_position(rob_fit_act,2), BENCHMARK_TYPE);
    end
    % Actualizar local best si posicion actual posee un merob_fit_actor fitness value
    for update_fit = 1:Q_Agents
        % Set actual fitness for each robot
        fitness_actual(update_fit) = fitness(actual_position(update_fit,1), actual_position(update_fit,2), BENCHMARK_TYPE);
        % Compare (i+1) fitness_actual with (i) fitness_local 
        if (fitness_actual(update_fit) < fitness_local(update_fit))
            best_local(update_fit,:) = actual_position(update_fit,1:2);
            fitness_local(update_fit) = fitness_actual(update_fit);
        end
    end
    % Update global values if (i+1) any local is better than the current global
    if (any(fitness_local < fitness_global_))
        [~,index1] = min(fitness_local);
        best_global = best_local(index1,:);
        fitness_global_ = fitness_local(index1);
    end
    % Generacion de valores de Parametros de Uniformidad (rho = [0,1])
    % rho1 = printRandoms(0, 1, 1);
    % rho2 = printRandoms(0, 1, 1);
    rho1 = randfrac();
    rho2 = randfrac();

    % Calculations of Time-Varying Inertia
    if INERTIA_TYPE == 0
        % Constant Inertia Parameter (Regular Functionality)
        w(:) = 0.8;
    elseif INERTIA_TYPE == 1
        % Linearly Decreasing Inertia Parameter (Poor Functionality)
        w(:) = w_max - (w_max - w_min) * iter / MAXiter;
    elseif INERTIA_TYPE == 2
        % Chaotic Inertia Parameter (Regular-Poor Functionality)
        zi = 0.2;
        zii = 4 * zi * (1 - zi);
        w(:) = (w_max - w_min) * ((MAXiter - iter) / MAXiter) * w_max * zii;
        iter = iter + 1;
    elseif INERTIA_TYPE == 3
        % Random Inertia Parameter (Good Functionality)
        w(:) = 0.5 + rand() / 2;
    elseif INERTIA_TYPE == 4
        % Exponential Inertia Parameter (Excellent Functionality)
        w(:) = w_min + (w_max - w_min) * exp((-1 * iter) / (MAXiter / 10));
        iter = iter + 1;
    end

    % Using Standard PSO Configuration for Path Planner (Adrob_fit_actusting velocity scaler)
    if USE_STANDARD_PSO == 1
        % Standard Configuration of PSO Scaling Parameters
        c1 = 2.05;
        c2 = 2.05;

        % Standard Configuration of PSO Constriction Parameter
        phi_T = c1 + c2;
        epsilon = 2.0 / abs(2 - phi_T - sqrt(phi_T^2 - 4 * phi_T));

        % PSO Velocity Scaler Configuration (Adrob_fit_actusted to replicate results without Standard PSO)
        V_scaler = 0.25;
        
    end

    % PSO Velocity Equation for calculating new agent velocity
    old_velocity = new_velocity;
    %new_velocity(vel,1) = epsilon * (w(vel) * old_velocity(vel,1) + c1 * rho1 * (best_local(vel,1) - actual_position(vel,1)) + c2 * rho2 * (best_global(1) - actual_position(vel,1)));
    %new_velocity(vel,2) = epsilon * (w(vel) * old_velocity(vel,2) + c1 * rho1 * (best_local(vel,2) - actual_position(vel,2)) + c2 * rho2 * (best_global(2) - actual_position(vel,2)));
    new_velocity = epsilon .* (w .* old_velocity + c1 .* rho1 .* (best_local - actual_position) + c2 .* rho2 .* (best_global - actual_position));
    % PSO Position Equation for calculating new agent position
    if mod(number_iteration, PSO_STEP) == 0 || number_iteration == 1
        new_position = actual_position + new_velocity * V_scaler;
        %new_position = actual_position + new_velocity * V_scaler;
    end
 % ---------------------------- CONTROLLER VARIABLES ---------------------------------
        % Calculation of distance errors
        e_x = new_position(:,1) - actual_position(:,1);      % Error in X position
        e_y = new_position(:,2) - actual_position(:,2);      % Error in Y position
        %e_p(vel) = sqrt(e_x(vel)^2 + e_y(vel)^2);                       % Magnitude of vectorial error
        e_t = [e_x, e_y];
        e_p = sqrt(sum(e_t.^2, 2));
        
        % E-Puck dimensions
        l = ROBOT_RADIUS;     % Distance from center to wheels in meters
        r = WHEEL_RADIUS;     % Wheel radius in meters
        a = ROBOT_RADIUS;     % Distance between center and disomorphism point (E-puck front)

        K = v0 * (1 - exp(-alpha_v .* e_p.^2)) ./ e_p;
        
        % Linear Velocity (applied with disomorphism matrix and 2*tanh(x) to limit velocities to E-Puck's MAX_SPEED (6.28 rad/s))
        %v(vel) = (2 * tanh((K / MAX_SPEED) * e_x(vel))) * cos((theta_o(vel)) * pi / 180) + ...
        %    (2 * tanh((K / MAX_SPEED) * e_y(vel))) * sin((theta_o(vel)) * pi / 180);

        % Angular Velocity (applied with disomorphism matrix and 2*tanh(x) to limit velocities to E-Puck's MAX_SPEED (6.28 rad/s))
        %w(vel) = (2 * tanh((K / MAX_SPEED) * e_x(vel))) * (-sin((theta_o(vel)) * pi / 180) / a) + ...
        %    (2 * tanh((K / MAX_SPEED) * e_y(vel))) * (cos((theta_o(vel)) * pi / 180) / a);
        
        theta_g = atan2(new_position(:,1) - actual_position(:,1), new_position(:,2) - actual_position(:,2));

        %if theta_o(vel) > 180
        %    orientation_angle(vel) = theta_o(vel) - 360;
        %else
        %    orientation_angle(vel) = theta_o(vel);
        %end

        % Calculation of angular distance error (Difference between -Z axis of robot and north, and angle of goal relative to robot and north)
        e_o = angdiff(theta_o,theta_g);

        % Angular Velocity PID
        e_D = e_o - e_old;
        E_o = E_o + e_o;
        e_old = e_o;
        
        if PID_CONTROLLER == 1
            w = KP * e_o + KI * E_o + KD * e_D;
            v = K.*e_p;
        end

        % If LQR_PID_COMBO is not being used, update errors in this block
        % ---------------------------- NONLINEAR ROBOT CONTROLS -----------------------------

        % Distance between robot's center and goal point
        % Limit orientation angle of the goal within [-pi, pi]
        % Simple Pose Controller for Robot
        % Lyapunov Pose Controller for Robot
        % Closed-loop Steering Controller
        % LQR Controller
        % LQI Controller
        
        % Termination of movement when close to the goal to avoid hard switches or robot vibrations at the goal
        %if abs(e_p(vel)) < 0.005
        %    v(vel) = 0;
        %    w(vel) = 0;
        %end

        % Transformation of motor speeds with differential drive model
        
        % Calculation of angular speeds of E-Puck motors depending on required linear and angular velocities
        phi_r = (v + w * l) / r;
        phi_l = (v - w * l) / r;
        phi_r = convangvel(phi_r, 'rad/s', 'rpm');
        phi_l = convangvel(phi_l, 'rad/s', 'rpm');
        % Truncate right motor rotation speeds to [-6.28, 6.28]
        for vel = 1:Q_Agents
            if phi_r(vel) > 0
                if phi_r(vel) > MAX_SPEED*2
                    phi_r(vel) = MAX_SPEED*2;
                end
            else
                if phi_r(vel) < -MAX_SPEED*2
                    phi_r(vel) = -MAX_SPEED*2;
                end
            end
            
            % Truncate left motor rotation speeds to [-6.28, 6.28]
            if phi_l(vel) > 0
                if phi_l(vel) > MAX_SPEED*2
                    phi_l(vel) = MAX_SPEED*2;
                end
            else
                if phi_l(vel) < -MAX_SPEED*2
                    phi_l(vel) = -MAX_SPEED*2;
                end
            end
        end    
            
        % Peaks cleaning
        disp(phi_l)
        disp(phi_r)
        for send_vel = first_agent:last_agent
            eval(['robotat_3pi_set_wheel_velocities(robot_' num2str(send_vel) ',phi_l(send_vel-first_agent+1),phi_r(send_vel-first_agent+1));']);
            %robotat_3pi_set_wheel_velocities(agents, phi_l(send_vel-first_agent+1), phi_r(send_vel-first_agent+1));
        end
        phi_l_hist = [phi_l_hist phi_l];
        phi_r_hist = [phi_r_hist phi_r];
        v_hist = [v_hist v];
        w_hist = [w_hist w];
            
end
%% Stop and disconnect robots for next run
for disc = first_agent:last_agent
    %sequence = 'eulxyz';
    eval(['robotat_3pi_force_stop(robot_' num2str(disc) ');']);
    eval(['robotat_3pi_disconnect(robot_' num2str(disc) ');']);
end