% MATLAB controller for Webots
% File:          e-puck_MPSO_m.m
% Date:          06/20/2023
% Description:   e-puck_MPSO migration to matlab
% Author:        Jonathan Menéndez Cardona 18023
% Modifications:  

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;
clear;
close all;
clc;
%wb_console_print(probando, WB_STDOUT)
load('Orientaciones.mat','bearing_deg');

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
MAX_SPEED = WHEEL_RADIUS * MAX_WHEEL_VELOCITY*3;
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
K_PROPORTIONAL = 0.5;
K_INTEGRAL = 0.1;
K_DERIVATIVE = 0.001;

%NKC Parameters (K_p > 0; K_b < 0; K_a > K_p) Kp = 0.01
K_DISTANCE = 0.1; 
K_alpha = 0.5;
K_beta = 0.05;

MAX_CHANGE = 1.00;

%% Robotat Configuration
robotat = robotat_connect();
%pos1 = robotat_get_pose(robotat,1,'eulxyz');

agents = cell(1,10);
first_agent = 7;  % first number of 3pi available
last_agent = 10;   % last number of 3pi available
Q_Agents = last_agent-first_agent+1;
for No_Agents = first_agent:last_agent
    agents{No_Agents} = robotat_3pi_connect(No_Agents);
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
while 1
    number_iteration = number_iteration + 1;
    disp(number_iteration);
    %position_robot((1:Q_Agents),:) = robotat_get_pose(robotat,(1:Q_Agents),'eulaxyz')
    
    if state == false
    position_robot = robotat_get_pose(robotat,(first_agent:last_agent),'eulzyx');
    actual_position = position_robot(:,1:2);   % Coordenada en eje Z de simulacion es coordenada X
    %actual_position(:,2) = position_robot(:,2);   % Coordenada en eje X de simulacion es coordenada Y
    best_local = actual_position;
    best_global = actual_position;
   
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
    position_robot((1:Q_Agents),:) = robotat_get_pose(robotat,(first_agent:last_agent),'eulazyx');
    
    %theta_o(1:Q_Agents) = (position_robot(1:Q_Agents)+90)*pi/180;
    north(1:Q_Agents,:) =  position_robot(1:Q_Agents,4);
    %north(1:Q_Agents)'
    rad(1:Q_Agents,1) = north(1:Q_Agents) - bearing_deg(first_agent:last_agent);
    tetha_o = rad;
    %disp(north(8))
    %disp(rad(8))
% ------------------------- ACTUALIZACION DE LOCAL AND GLOBAL BESTS -------------------------    
% 
    % Calcular Fitness Value de posicion actiaul
    actual_position = position_robot;
    %actual_position = position_robot;
    for j = 1:Q_Agents
    fitness_actual(j) = fitness(actual_position(j,1), actual_position(j,2), BENCHMARK_TYPE);
    end
    % Actualizar local best si posicion actual posee un mejor fitness value
    for len1 = 1:Q_Agents
        % Set actual fitness for each robot
        fitness_actual(len1) = fitness(actual_position(len1,1), actual_position(len1,2), BENCHMARK_TYPE);
        % Compare (i+1) fitness_actual with (i) fitness_local 
        if (fitness_actual(len1) < fitness_local(len1))
            best_local(len1,:) = actual_position(len1,1:2);
            fitness_local(len1) = fitness_actual(len1);
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
        w(1:Q_Agents) = 0.8;
    elseif INERTIA_TYPE == 1
        % Linearly Decreasing Inertia Parameter (Poor Functionality)
        w(1:Q_Agents) = w_max - (w_max - w_min) * iter / MAXiter;
    elseif INERTIA_TYPE == 2
        % Chaotic Inertia Parameter (Regular-Poor Functionality)
        zi = 0.2;
        zii = 4 * zi * (1 - zi);
        w(1:Q_Agents) = (w_max - w_min) * ((MAXiter - iter) / MAXiter) * w_max * zii;
        iter = iter + 1;
    elseif INERTIA_TYPE == 3
        % Random Inertia Parameter (Good Functionality)
        w(1:Q_Agents) = 0.5 + rand() / 2;
    elseif INERTIA_TYPE == 4
        % Exponential Inertia Parameter (Excellent Functionality)
        w(1:Q_Agents) = w_min + (w_max - w_min) * exp((-1 * iter) / (MAXiter / 10));
        iter = iter + 1;
    end

    % Using Standard PSO Configuration for Path Planner (Adjusting velocity scaler)
    if USE_STANDARD_PSO == 1
        % Standard Configuration of PSO Scaling Parameters
        c1 = 2.05;
        c2 = 2.05;

        % Standard Configuration of PSO Constriction Parameter
        phi_T = c1 + c2;
        epsilon = 2.0 / abs(2 - phi_T - sqrt(phi_T^2 - 4 * phi_T));

        % PSO Velocity Scaler Configuration (Adjusted to replicate results without Standard PSO)
        V_scaler = 0.25;

        if TUC_CONTROLLER == 1
            V_scaler = 0.625;
        end
        if PID_CONTROLLER == 1
            V_scaler = 7.8125;
        end
    end
    for vel = 1:Q_Agents
        % PSO Velocity Equation for calculating new agent velocity
        old_velocity(vel,:) = new_velocity(vel,:);
        new_velocity(vel,1) = epsilon * (w(vel) * old_velocity(vel,1) + c1 * rho1 * (best_local(vel,1) - actual_position(vel,1)) + c2 * rho2 * (best_global(1) - actual_position(vel,1)));
        new_velocity(vel,2) = epsilon * (w(vel) * old_velocity(vel,2) + c1 * rho1 * (best_local(vel,2) - actual_position(vel,2)) + c2 * rho2 * (best_global(2) - actual_position(vel,2)));
        % PSO Position Equation for calculating new agent position
        if mod(number_iteration, PSO_STEP) == 0 || number_iteration == 1
            new_position(vel,1) = actual_position(vel,1) + new_velocity(vel,1) * V_scaler;
            new_position(vel,2) = actual_position(vel,2) + new_velocity(vel,2) * V_scaler;
        end

 % ---------------------------- CONTROLLER VARIABLES ---------------------------------
        % Calculation of distance errors
        e_x(vel) = new_position(vel,1) - actual_position(vel,1);      % Error in X position
        e_y(vel) = new_position(vel,2) - actual_position(vel,2);      % Error in Y position
        e_p(vel) = sqrt(e_x(vel)^2 + e_y(vel)^2);                       % Magnitude of vectorial error
    
        % E-Puck dimensions
        l = ROBOT_RADIUS;     % Distance from center to wheels in meters
        r = WHEEL_RADIUS;     % Wheel radius in meters
        a = ROBOT_RADIUS;     % Distance between center and disomorphism point (E-puck front)

        K = 3.12 * (1 - exp(-2 * power(e_p(vel), 1))) / e_p(vel);


        % Linear Velocity (applied with disomorphism matrix and 2*tanh(x) to limit velocities to E-Puck's MAX_SPEED (6.28 rad/s))
        v(vel) = (2 * tanh((K / MAX_SPEED) * e_x(vel))) * cos((theta_o(vel)) * pi / 180) + ...
            (2 * tanh((K / MAX_SPEED) * e_y(vel))) * sin((theta_o(vel)) * pi / 180);

        % Angular Velocity (applied with disomorphism matrix and 2*tanh(x) to limit velocities to E-Puck's MAX_SPEED (6.28 rad/s))
        w(vel) = (2 * tanh((K / MAX_SPEED) * e_x(vel))) * (-sin((theta_o(vel)) * pi / 180) / a) + ...
            (2 * tanh((K / MAX_SPEED) * e_y(vel))) * (cos((theta_o(vel)) * pi / 180) / a);
        
        theta_g(vel) = atan2(new_position(vel,1) - actual_position(vel,1), new_position(vel,2) - actual_position(vel,2));

        if theta_o(vel) > 180
            orientation_angle(vel) = theta_o(vel) - 360;
        else
            orientation_angle(vel) = theta_o(vel);
        end

        % Calculation of angular distance error (Difference between -Z axis of robot and north, and angle of goal relative to robot and north)
        e_o(vel) = atan2(sin(theta_g(vel) - (orientation_angle(vel) * pi / 180)), cos(theta_g(vel) - (orientation_angle(vel) * pi / 180)));

        % Angular Velocity PID
        e_D(vel) = e_o(vel) - e_old(vel);
        E_o(vel) = E_old(vel) + e_o(vel);
        
        if PID_CONTROLLER == 1
            w(vel) = KP * e_o(vel) + KI * E_o(vel) + KD * e_D(vel);
        end

        % If LQR_PID_COMBO is not being used, update errors in this block
        if LQR_PID_COMBO == 0
            e_old(vel) = e_o(vel);
            E_old(vel) = E_o(vel);
        end

        % ---------------------------- NONLINEAR ROBOT CONTROLS -----------------------------

        % Distance between robot's center and goal point
        rho_p(vel) = e_p(vel);

        % Limit angle between robot's front axis (-Z_R) and vector towards the goal within range [-pi, pi]
        alpha(vel) = - (theta_o(vel) * pi / 180) + atan2(e_y(vel), e_x(vel));
        if alpha(vel) < -pi
            alpha(vel) = alpha(vel) + 2 * pi;
        elseif alpha(vel) > pi
            alpha(vel) = alpha(vel) - 2 * pi;
        end

        % Limit orientation angle of the goal within [-pi, pi]
        beta(vel) = - (theta_o(vel) * pi / 180) - alpha(vel);
        if beta(vel) < -pi
            beta(vel) = beta(vel) + 2 * pi;
        elseif beta(vel) > pi
            beta(vel) = beta(vel) - 2 * pi;
        end
        % Simple Pose Controller for Robot
        if NKC_CONTROLLER_1 == 1
            v(vel) = K_RHO * rho_p(vel);
            w(vel) = K_A * alpha(vel) + K_B * beta(vel);
            if alpha(vel) <= -pi/2 || alpha(vel) > pi/2
                v(vel) = -v(vel);
            end
        end

        % Lyapunov Pose Controller for Robot
        if NKC_CONTROLLER_2 == 1
            v(vel) = K_RHO * rho_p(vel) * cos(alpha(vel));
            w(vel) = K_RHO * sin(alpha(vel)) * cos(alpha(vel)) + K_A * alpha(vel);
            if alpha(vel) <= -pi/2 || alpha(vel) > pi/2
                v(vel) = -v(vel);
            end
        end

        % Closed-loop Steering Controller
        if NKC_CONTROLLER_3 == 1
            k_1 = 1;
            k_2 = 10;
            w(vel) = -(2.0/5) * (v(vel)/rho_p(vel)) * (k_2 * (-alpha(vel) - atan(-k_1 * beta(vel))) + (1 + k_1 / (1 + (k_1 * beta(vel))^2)) * sin(-alpha(vel)));
            
% % Linear Velocity Selector
%             lambda = 2;
%             b_parameter = 0.4;
%             curve = -(1/rho_p)*(k_2*(-alpha(vel) - atan(-k_1*beta(vel))) + (1 + k_1/(1 + pow(k_1*beta(vel),2)))*sin(-alpha(vel)));
%             v = 0.05 / (1 + b_parameter * abs(curve)^2);
%             w = curve * v;
        end

        % LQR Controller
        if LQR_CONTROLLER == 1
            K_x = 0.1;
            K_y = 0.1;
            
            u_1(vel) = -K_x * (actual_position(vel,1) - new_position(vel,1));
            u_2(vel) = -K_y * (actual_position(vel,2) - new_position(vel,2));
            
            v(vel) = u_1 * cos((theta_o(vel)) * pi / 180) + u_2(vel) * sin((theta_o(vel)) * pi / 180);
            w(vel) = (-u_1 * sin((theta_o(vel)) * pi / 180) + u_2(vel) * cos((theta_o(vel)) * pi / 180)) / l;
            
            if LQR_PID_COMBO == 1
                Ti = 3; % Constant to reduce oscillations for PID+LQR
                w(vel) = KP * e_o(vel) + (KI / Ti) * E_o(vel) + KD * e_D(vel);
                e_old(vel) = e_o(vel);
                E_old(vel) = E_o(vel);
            end
        end

        % LQI Controller
        if LQI_CONTROLLER == 1
%             Klqr_x = 0.2127;
%             Klqr_y = 0.2127;
%             Klqi_x = -0.0224;
%             Klqi_y = -0.0224;
            
            Klqr_x = 0.52127;
            Klqr_y = 0.52127;
            Klqi_x = -0.50224;
            Klqi_y = -0.50224;
            
            % Dampers for LQI control
            bv_p = 0.95;
            bv_i = 0.01;
            
            % LQI Linear Quadratic Integrator Controller [u = -K*e - Ki*Ei]
            u_1(vel) = -Klqr_x * (1 - bv_p) * (actual_position(vel,1) - new_position(vel,1)) - Klqi_x * XI_X(vel);
            u_2(vel) = -Klqr_y * (1 - bv_p) * (actual_position(vel,2) - new_position(vel,2)) - Klqi_y * XI_Y(vel);
            
            % Numerical integration of error between current position and global best of swarm
            XI_X(vel) = XI_X(vel) + (best_global(1) - actual_position(vel,1)) * delta_t;
            XI_Y(vel) = XI_Y(vel) + (best_global(2) - actual_position(vel,2)) * delta_t;
            
            % Braking integrator to prevent robot position oscillations
            XI_X(vel) = (1 - bv_i) * XI_X(vel);
            XI_Y(vel) = (1 - bv_i) * XI_Y(vel);
            
            % Mapping LQI velocities to differential robot velocities through diffeomorphism
            v(vel) = u_1(vel) * cos((theta_o(vel)) * pi / 180) + u_2(vel) * sin((theta_o(vel)) * pi / 180);
            w(vel) = (-u_1(vel) * sin((theta_o(vel)) * pi / 180) + u_2(vel) * cos((theta_o(vel)) * pi / 180)) / l;
        end
        

        % Termination of movement when close to the goal to avoid hard switches or robot vibrations at the goal
        if abs(e_p(vel)) < 0.005
            v(vel) = 0;
            w(vel) = 0;
        end

        % Transformation of motor speeds with differential drive model
        
        % Calculation of angular speeds of E-Puck motors depending on required linear and angular velocities
        phi_r(vel) = -(v(vel) + w(vel) * l) / r;
        phi_l(vel) = -(v(vel) - w(vel) * l) / r;
        
        % Truncate right motor rotation speeds to [-6.28, 6.28]
        if phi_r(vel) > 0
            if phi_r(vel) > MAX_SPEED
                phi_r(vel) = MAX_SPEED;
            end
        else
            if phi_r(vel) < -MAX_SPEED
                phi_r(vel) = -MAX_SPEED;
            end
        end
    
        % Truncate left motor rotation speeds to [-6.28, 6.28]
        if phi_l(vel) > 0
            if phi_l(vel) > MAX_SPEED
                phi_l(vel) = MAX_SPEED;
            end
        else
            if phi_l(vel) < -MAX_SPEED
                phi_l(vel) = -MAX_SPEED;
            end
        end

        
        % Peaks cleaning
        if HARDSTOP_FILTER == 1
            if sqrt((phi_r(vel) - PhiR_old(vel))^2) > MAX_CHANGE
                phi_r(vel) = (phi_r(vel) + 2 * PhiR_old(vel)) / 3;
            end
            PhiR_old(vel) = phi_r(vel);
            if sqrt((phi_l(vel) - PhiL_old(vel))^2) > MAX_CHANGE
                phi_l(vel) = (phi_l(vel) + 2 * PhiL_old(vel)) / 3;
            end
            PhiL_old(vel) = phi_l(vel);
        end
        
        disp(phi_l(2))
        disp(phi_r(2))
        for send_vel = first_agent:last_agent
            robotat_3pi_set_wheel_velocities(agents{send_vel}, phi_l(send_vel-6), phi_r(send_vel-6));
        end
        phi_l_hist = [phi_l_hist phi_l];
        phi_r_hist = [phi_r_hist phi_r];
        v_hist = [v_hist v];
        w_hist = [w_hist w];
        
    end
end
%%
% fitness1 = fitness(1,2,4)
% if find(aaa([1:2-1,2+1:end]) < 3)
%     strin = "hello";
% end
%% Functions
