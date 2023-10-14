function [phi,x,y,u]  = PID_controller1(obj_tcp,agent,ang_diff,tray,k)
%PID_CONTROLLER1 Summary of this function goes here

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
eO_D = 0;
eO_1 = 0;
EO = 0;
%   Detailed explanation goes here
xi = robotat_get_pose(obj_tcp,agent,'eulxyz');
x = xi(1); y = xi(2); theta = (xi(6)-ang_diff)*pi/180;
xg = tray(k,1);
yg = tray(k,2);
e = [xg - x; yg - y];
thetag = atan2(e(2), e(1));
eP = norm(e);
eO = angdiff(theta,thetag);
kP = v0 * (1-exp(-alpha*eP^2)) / eP;
v = kP*eP;
eO_D = eO - eO_1;
EO = EO + eO;
w = kpO*eO + kiO*EO + kdO*eO_D;
eO_1 = eO;

u = [v; w];

phi_R = (v+w*ell)/radio;
phi_L = (v-w*ell)/radio;
phi_L = convangvel(phi_L, 'rad/s', 'rpm');
phi_R = convangvel(phi_R, 'rad/s', 'rpm');

if phi_L > 50
    phi_L = 50;
end
if phi_L < -50
    phi_L = -50;
end
if phi_R > 50
    phi_R = 50;
end
if phi_R < -50
    phi_R = -50;
end

phi = [phi_L,phi_R];
end

