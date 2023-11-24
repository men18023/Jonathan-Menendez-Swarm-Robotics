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
% Define the goal position
% goal_x = 0;
% goal_y = 0;

% Define control gains
% Kp_linear = 0.5;
% Kp_angular = 0.05;
% 
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

%% Movimiento controlado

goal_1 = robotat_get_pose(robotat,1,ang_seq);
goal = goal_1(1:2);
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
    
    spmd(Q_Agents)
        
    end
    
%     for agent_vel = first_agent:last_agent
%            numb = agent_vel-first_agent+1;
%            [phi,xi,yi,u] = PID_controller1(robotat,agent_vel,bearing_new(agent_vel),tray{numb},k);
%            eval(['robotat_3pi_set_wheel_velocities(robot_' num2str(agent_vel) ',phi(1),phi(2))']);
%            %display(phi)
%            %eval(['phi_L{' num2str(numb) '}' '=[phi_L{' num2str(numb) '}; phi(1)']);
%            %eval(['phi_R{' num2str(numb) '}' '=[phi_R{' num2str(numb) '}; phi(2)']);
%            eval(['trajectory{' num2str(numb) '}' '=[trajectory{' num2str(numb) '}; xi, yi];']);
%            %eval(['u_vw{' num2str(numb) '}' '=[trajectory{' num2str(numb) '}; u];']);
%            %trajectory = [trajectory; [xi(1), xi(2)]];
%     end
%     trajectory1 = [trajectory1; [xi(1), xi(2)]];
%     v_hist = [v_hist; v];
%     w_hist = [w_hist; w];
%     rwheel_hist = [rwheel_hist; phi_R];
%     lwheel_hist = [lwheel_hist; phi_L];
%     goal = [x_tray7(end), y_tray7(end)];
%     save('analysis.mat', 'trajectory', 'v_hist', 'w_hist', 'rwheel_hist', 'lwheel_hist', 'goal','-append')
    k=k+1;
end
%%
%tEnd = toc(tStart);      % pair 2: toc
pause(1);
for disc = first_agent:last_agent
    %sequence = 'eulxyz';
    eval(['robotat_3pi_force_stop(robot_' num2str(disc) ');']);
    eval(['robotat_3pi_disconnect(robot_' num2str(disc) ');']);
end