function [phi_L,phi_R, trajectory_x, trajectory_y, v, w]  = PID_controller1(xi,offset,traj)
%PID_CONTROLLER1 Summary of this function goes here 
% Propiedades físicas del robot
MAX_WHEEL_VELOCITY = 800;   % maximum rpm available for each wheel
WHEEL_RADIUS = 32/2000;     % radius in meters
MAX_SPEED = WHEEL_RADIUS * MAX_WHEEL_VELOCITY;
DISTANCE_FROM_CENTER = 96/2000; % distance from center to wheels in meters
%radio=WHEEL_RADIUS/2;
limiter = 50;     % controlled max rpm 

% Posición
v0 = MAX_SPEED/6; % initial speed
alpha = 0.9;   

% PID Orientación
kpO = 0.7;
kiO = 0.01;
kdO = 0.0001;
eO_D = 0;
eO_1 = 0;
EO = 0;
%xi = robotat_get_pose(obj_tcp,Agent,'eulxyz');
x = xi(1); y = xi(2);  %theta = (xi(6)-90)*pi/180;
theta = deg2rad(xi(6)+offset);
xg = traj(1);
yg = traj(2);
e = [xg - x; yg - y];
thetag = atan2(e(2), e(1));
trajectory_x = xi(1);
trajectory_y = xi(2);
eP = norm(e);
eO = angdiff(theta,thetag);
%ee = [eP;eO];

% Lineal velocity control
kP = v0 * (1-exp(-alpha*eP^2)) / eP;
v = kP*eP;
if eP < 0.05
    v = 0;
end
% Angular velocity control
eO_D = eO - eO_1;
EO = EO + eO;
w = kpO*eO + kiO*EO + kdO*eO_D;
eO_1 = eO;

% Combination of controllers
u = [v; w];

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

phi = [phi_L,phi_R];
end

