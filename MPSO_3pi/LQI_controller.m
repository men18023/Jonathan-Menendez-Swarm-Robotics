function [phi,x,y,u]  = LQI_controller(obj_tcp,agent,ang_diff,tray,k)
%PID_CONTROLLER1 Summary of this function goes here

% Propiedades físicas del robot
MAX_WHEEL_VELOCITY = 800;
WHEEL_RADIUS = 0.032;
MAX_SPEED = WHEEL_RADIUS * MAX_WHEEL_VELOCITY;
ell=96/2000;
radio=32/2000;

%%LQI
sigma=0;
Cr = eye(2);
B = eye(2);
R = eye(2);
A = zeros(2); 
Dr = zeros(2);
AA = [A, zeros(size(Cr')); Cr, zeros(size(Cr,1))];
BB = [B; Dr];
QQ = eye(size(A,1) + size(Cr,1));
QQ(3,3) = 2.2; QQ(4,4) = 0.7;
Klqi = lqr(AA, BB, QQ, R);

xi = robotat_get_pose(obj_tcp,agent,'eulxyz');
x = xi(1); y = xi(2); theta = (xi(6)-ang_diff)*pi/180;
pos = [x;y];
%ee = tray(k,1)-xi(1);
thetag = atan2(theta(2), theta(1));
pos = [pos;thetag];
sigma = sigma + (Cr*pos - tray(k,:))*0.001;
mu = -Klqi*[pos; sigma];
u=finv(pos,mu);

phi_R = (u(1) - u(2)*ell) / radio;
phi_L = (u(1) + u(2)*ell) / radio;
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

