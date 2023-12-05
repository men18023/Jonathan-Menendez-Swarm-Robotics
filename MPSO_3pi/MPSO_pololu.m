% MATLAB controller for Webots
% File:          MPSO_3pi.m
% Date:          06/20/2023
% Description:   e-puck_MPSO migration to matlab
% Author:        Jonathan Menéndez Cardona 18023
% Modifications:  

% Cleans workspace and command window 
if exist('robotat', 'var')
    robotat_disconnect(robotat);
end
pause(0.5)
clear;
close all;
clc;
% Loads bearing angles 
load('Orientaciones.mat','bearing_deg','bearing_new');

% Connects to robotat rerver and available agents
% robotat = robotat_connect();
if ~exist('robotat', 'var')
    robotat = robotat_connect();
end
ang_seq = 'eulxyz';  % sequence for all get_pose 
first_agent = 2;     % first number of 3pi available
last_agent = 7;      % last number of 3pi available
Q_Agents = last_agent-first_agent+1;
for con = first_agent:last_agent %first_agent:last_agent
    pause(0.1)
    eval(['robot_' num2str(con) '=robotat_3pi_connect(con);']);
end

%% Initialize PSO variables
%TUC_CONTROLLER = 0;
PID_CONTROLLER = 1;
%NKC_CONTROLLER_1 = 0;
%NKC_CONTROLLER_2 = 0;
%NKC_CONTROLLER_3 = 0;
%LQR_CONTROLLER = 0;
%LQI_CONTROLLER = 0;

%HARDSTOP_FILTER = 0;

%LQR_PID_COMBO = 0;

%USE_BEARING = 0;

TIME_STEP = 64;

BENCHMARK_TYPE = 0;
TIME_DELTA = 0.2;

PSO_STEP = 1;

USE_STANDARD_PSO = 1;

INERTIA_TYPE = 4;
% PSO Parameters (Default: Constriction = 0.8, Local Weight = 2; Global Weight = 10)
CONSTRICTION_FACTOR = 0.8;
COGNITIVE_WEIGTH = 2;
SOCIAL_WEIGTH = 10;

% Variables de Proportional Integral Derivative controller (PID)
% KP = K_PROPORTIONAL; % PID proportional gain
% KI = K_INTEGRAL; % PID integral gain
% KD = K_DERIVATIVE; % PID derivative gain
e_old = zeros(Q_Agents,1); % Previous PID error
E_old = zeros(Q_Agents,1); % Previous accumulated PID error
e_o = zeros(Q_Agents,1); % Current PID error
E_o = zeros(Q_Agents,1); % Current accumulated PID error
e_D = zeros(Q_Agents,1); % Differential error

% Variables de Non-lineal Kinematics controller (NKC)
alpha = zeros(Q_Agents,1); % Angle between robot-target vector and robot's front vector
beta = zeros(Q_Agents,1); % Target's orientation angle in inertial frame
rho_p = zeros(Q_Agents,1); % Euclidean distance between robot centroid and target
% K_RHO = K_DISTANCE; % Distance control gain
% K_A = K_alpha; % Orientation control gain
% K_B = -K_beta; % Target rotation control gain

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

number_iteration = 0;
MAX_ITER = 100; % max iteration available for trajectory execution.

%e_hist = zeros(2, length(traj)); 
trajectory_hist = zeros(MAX_ITER,2*length(1:last_agent)); 
v_hist = zeros(MAX_ITER, length(1:last_agent)); 
w_hist = zeros(MAX_ITER, length(1:last_agent)); 
rwheel_hist = zeros(MAX_ITER, length(1:last_agent)); 
lwheel_hist = zeros(MAX_ITER, length(1:last_agent)); 
%phi_hist = zeros(2,length(traj));

index1 = 0;
state = false;
pause(2)

iteration = 0;

% Initialize all history and algoritm variables.
% trajectory = cell(1, Q_Agents);
% u_vw = cell(1, Q_Agents);
% phi_L = cell(1, Q_Agents);
% phi_R = cell(1, Q_Agents);
% Generate and store the arrays in the cell array
% for i = 1:Q_Agents
%     % Create a 2x1 vector filled with initial values
%     vector = robotat_get_pose(robotat,i,ang_seq);
%     vector0 = zeros(1,2);
%     value = 0;
%     % Store the vector in the cell array
%     trajectory{i} = vector;
%     u_vw{i} = vector;
%     phi_L{i} = value;
%     phi_L{i} = value;
% end

offset = zeros(10,1);
for b = 1:11
    bearing = abs(bearing_deg(b)) + 90;
    
    if (bearing > 180)
        bearing = bearing - 360;
    elseif (bearing < -180)
        bearing = bearing + 360;
    end
    offset(b) = bearing; 
end
filename = strcat("analysis_",num2str(BENCHMARK_TYPE),"_",num2str(INERTIA_TYPE));
filename = strcat(filename,'.mat');

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
w_min = 0.1; % PSO inertia minimum
w_max = 0.4; % PSO inertia maximum
MAXiter = 10000; % Maximum number of iterations
iter = 0; % Iteration counter

new_pos_hist = [];
%% Movimiento controlado


% Ciclo decontrol

k=1;
%%
while number_iteration < MAX_ITER
    %tStart = tic;
    number_iteration = number_iteration + 1;
    disp(number_iteration);
    %disp(k)
    % Configurar valores iniciales de PSO local y global (Ejecutar solo una vez)
    if state == false
        position_robot = robotat_get_pose(robotat,(first_agent:last_agent),'eulzyx');
        actual_position = position_robot(:,1:2);   % Coordenadas x,y
        %actual_position(:,2) = position_robot(:,2);   % Coordenada en eje X de simulacion es coordenada Y
        best_local = actual_position;
        norms = vecnorm(actual_position, 2, 2);
        [min_norm, min_norm_row] = min(norms);
        best_global = actual_position(min_norm_row,:);
        
        % Calcular Fitness Value
        for i = 1:Q_Agents
            fitness_local(i) = fitness(actual_position(i,1), actual_position(i,2), BENCHMARK_TYPE);
            fitness_global(i) = fitness(actual_position(i,1), actual_position(i,2), BENCHMARK_TYPE);
        end
        fitness_global = min(fitness_global);
        state = true;
    end
    
    position_robot = robotat_get_pose(robotat,(first_agent:last_agent),'eulazyx');
    % ------------------------- ACTUALIZACION DE LOCAL AND GLOBAL BESTS -------------------------
    
    % Calcular Fitness Value de posicion actual
    actual_position = position_robot(:,1:2);
    % actual_position = position_robot;
    
    for j = 1:Q_Agents
        fitness_actual(j) = fitness(actual_position(j,1), actual_position(j,2), BENCHMARK_TYPE);
        if (fitness_actual(j) < fitness_local(j))
            best_local(j,:) = actual_position(j,:);
            fitness_local(j) = fitness_actual(j);
        end
    end
    % Update global values if (i+1) any local is better than the current global
    if (any(fitness_local < fitness_global))
        [~,index1] = min(fitness_local);
        best_global = best_local(index1,:);
        fitness_global = fitness_local(index1);
    end
    
    %if norm(best_global-actual_position(3,:))<0.2
    %    break;
    %end
    % ---------------------------------- MPSO ALGORITHM -----------------------------------
    rho1 = rand(1);
    rho2 = rand(1);
    
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
        c1 = 1;
        c2 = 7;
        
        % Standard Configuration of PSO Constriction Parameter
        phi_T = c1 + c2;
        epsilon = 2.0 / abs(2 - phi_T - sqrt(phi_T^2 - 4 * phi_T));
        epsilon = epsilon;
        % PSO Velocity Scaler Configuration (Adjusted to replicate results without Standard PSO)
        V_scaler = 0.5;

        if PID_CONTROLLER == 0
            V_scaler = 5.0; 
        end
    end
    
    old_velocity = new_velocity;
    new_velocity(:,1) = epsilon .* (w(:) .* old_velocity(:,1) + c1 * rho1 .* (best_local(:,1) - actual_position(:,1)) + c2 * rho2 .* (best_global(1) - actual_position(:,1)));
    new_velocity(:,2) = epsilon .* (w(:) .* old_velocity(:,2) + c1 * rho1 .* (best_local(:,2) - actual_position(:,2)) + c2 * rho2 .* (best_global(2) - actual_position(:,2)));
%     if mod(number_iteration, PSO_STEP) == 0 || number_iteration == 1
    new_position(first_agent:last_agent,1) = actual_position(:,1) + new_velocity(:,1) * V_scaler;
    new_position(first_agent:last_agent,2) = actual_position(:,2) + new_velocity(:,2) * V_scaler;
    new_pos_hist = [new_pos_hist [new_position(:,1);new_position(:,2)]];
%     end
    
    xi(first_agent:last_agent,:) = robotat_get_pose(robotat,first_agent:last_agent,'eulxyz');
    parfor agent_vel = first_agent:last_agent
        %pause(0.1)
        numb = agent_vel - first_agent + 1;
        try
        [phi_L,phi_R, trajectory_x, trajectory_y, uv, uw] = PID_controller1(xi(agent_vel,:), offset(agent_vel), new_position(agent_vel, :));
        catch
        end
        %Do something with phi, e.g., save it to an array
         phiL_array(:,agent_vel) = phi_L;
         phiR_array(:,agent_vel) = phi_R;
         %trajectory_hist(:,agent_vel) = trajectory_x(:,1);
         traj_x(:,agent_vel) = trajectory_x;
         traj_y(:,agent_vel) = trajectory_y;
         v_array(:,agent_vel) = uv;
         w_array(:,agent_vel) = uw;
        %robotat_3pi_set_wheel_velocities(
    end
%     phi2 = PID_controller1(robotat, 2, offset(2), new_position(2, :));
%     phi3 = PID_controller1(robotat, 3, offset(3), new_position(3, :));
%     phi4 = PID_controller1(robotat, 4, offset(4), new_position(4, :));
%     phi5 = PID_controller1(robotat, 5, offset(5), new_position(5, :));
%     phi6 = PID_controller1(robotat, 6, offset(6), new_position(6, :));
%     phi7 = PID_controller1(robotat, 7, offset(7), new_position(7, :));
%     phi8 = PID_controller1(robotat, 8, offset(8), new_position(8, :));
    %phi9 = PID_controller1(robotat, 9, offset(9), new_position(9, :));
%     %phi5 = PID_controller1(robotat, 5, offset(5), new_position(5, :));
% 
%     %disp(phi_array)
    %trajectory(number_iteration,);
    trajectory_hist(number_iteration,:) = [traj_x traj_y];
    v_hist(number_iteration,:) = v_array;
    w_hist(number_iteration,:) = w_array;
    rwheel_hist(number_iteration,:) =  phiL_array;
    lwheel_hist(number_iteration,:) =  phiR_array;
    for i = first_agent:last_agent
    eval(['robotat_3pi_set_wheel_velocities(robot_' num2str(i) ',phiL_array(' num2str(i) '),phiR_array(' num2str(i) '))']);
    end
% 
%     eval(['robotat_3pi_set_wheel_velocities(robot_' num2str(2) ',phiL_array(2),phiR_array(2))']);
%     eval(['robotat_3pi_set_wheel_velocities(robot_' num2str(3) ',phiL_array(3),phiR_array(3))']);
%     eval(['robotat_3pi_set_wheel_velocities(robot_' num2str(4) ',phiL_array(4),phiR_array(4))']);
%     eval(['robotat_3pi_set_wheel_velocities(robot_' num2str(5) ',phiL_array(5),phiR_array(5))']);
%     eval(['robotat_3pi_set_wheel_velocities(robot_' num2str(6) ',phiL_array(6),phiR_array(6))']);
%     eval(['robotat_3pi_set_wheel_velocities(robot_' num2str(7) ',phiL_array(7),phiR_array(7))']);
%     eval(['robotat_3pi_set_wheel_velocities(robot_' num2str(8) ',phiL_array(8),phiR_array(8))']);

    %eval(['robotat_3pi_set_wheel_velocities(robot_' num2str(9) ',phi9(1),phi9(2))']);
%     eval(['robotat_3pi_set_wheel_velocities(robot_' num2str(7) ',phi_array(7,1),phi_array(7,2))']);
%     eval(['robotat_3pi_set_wheel_velocities(robot_' num2str(8) ',phi_array(8,1),phi_array(8,2))']);
%     eval(['robotat_3pi_set_wheel_velocities(robot_' num2str(9) ',phi_array(9,1),phi_array(9,2))']);
    %     for agent_vel = first_agent:last_agent-1
% %         spmd
% %             % Generate a single random number for each worker
% %             [phi,xi,yi,u,phi] = PID_controller1(robotat,agent_vel,bearing_new(agent_vel),tray{numb},k);
% %             %num(labindex) = randomNumber;
% %             %combinedResult = gcat(num(labindex), 1, 1);
% %         end
%         numb = agent_vel-first_agent+1;
%         %[phi,trajectory,u] = PID_controller1(robotat,agent_vel,offset(agent_vel),new_position(numb,:),Q_Agents);
%         pause(0.1)
%         phi = PID_controller_parallel(robotat,agent_vel,offset,new_position(numb,:));
%         eval(['robotat_3pi_set_wheel_velocities(robot_' num2str(agent_vel) ',phi(1),phi(2))']);
%         %display(phi)
%         %eval(['phi_L{' num2str(numb) '}' '=[phi_L{' num2str(numb) '}; phi(1)']);
%         %eval(['phi_R{' num2str(numb) '}' '=[phi_R{' num2str(numb) '}; phi(2)']);
%         %eval(['trajectory{' num2str(numb) '}' '=[trajectory{' num2str(numb) '}; xi, yi];']);
%         %eval(['u_vw{' num2str(numb) '}' '=[trajectory{' num2str(numb) '}; u];']);
%         %trajectory = [trajectory; [xi(1), xi(2)]];
%     end
%     trajectory1 = [trajectory1; [xi(1), xi(2)]];
%     v_hist = [v_hist; v];
%     w_hist = [w_hist; w];
%     rwheel_hist = [rwheel_hist; phi_R];
%     lwheel_hist = [lwheel_hist; phi_L];
%     goal = [x_tray7(end), y_tray7(end)];
    k=k+1;
    save(filename, 'first_agent', 'last_agent', 'Q_Agents', 'trajectory_hist', 'v_hist', 'w_hist', 'rwheel_hist', 'lwheel_hist', 'best_global','-append')

end
  
%%
%tEnd = toc(tStart);      % pair 2: toc
pause(1);
for disc = first_agent:last_agent %first_agent:last_agent
    %sequence = 'eulxyz';
    eval(['robotat_3pi_force_stop(robot_' num2str(disc) ');']);
    eval(['robotat_3pi_disconnect(robot_' num2str(disc) ');']);
end