% Initialize robotat 
clear;
close all;
robotat = robotat_connect();
 
% Define robot
%r7 = robotat_3pi_connect(7);
robot7 = robotat_get_pose(robotat,7,'eulzyx');

%% Matriz de obstaculos
% Robots móviles como obstaculos
o6 = robotat_get_pose(robotat,1,'ZYX');
o7 = robotat_get_pose(robotat,2,'ZYX');
o9 = robotat_get_pose(robotat,3,'ZYX');
o10 = robotat_get_pose(robotat,4,'ZYX');
o11 = robotat_get_pose(robotat,5,'ZYX');
o12 = robotat_get_pose(robotat,6,'ZYX');
o13 = robotat_get_pose(robotat,8,'ZYX');
o14 = robotat_get_pose(robotat,9,'ZYX');
o15 = robotat_get_pose(robotat,10,'ZYX');

% Obstaculos adecionales del proyecto 2
 o16 = robotat_get_pose(robotat,11,'ZYX');
 o17 = robotat_get_pose(robotat,12,'ZYX');
 o18 = robotat_get_pose(robotat,13,'ZYX');
 o19 = robotat_get_pose(robotat,14,'ZYX');
 o20 = robotat_get_pose(robotat,15,'ZYX');
 o21 = robotat_get_pose(robotat,16,'ZYX');
 o22 = robotat_get_pose(robotat,17,'ZYX');
 
% Obstaculos 
o1 = robotat_get_pose(robotat,18,'ZYX');
o2 = robotat_get_pose(robotat,19,'ZYX');
o3 = robotat_get_pose(robotat,20,'ZYX');
o4 = robotat_get_pose(robotat,21,'ZYX');
o5 = robotat_get_pose(robotat,22,'ZYX');

% se genera la matriz con todas las posiciones en x y en y de todos los
% obstáculos
Mobsc = [o1(1:2);o2(1:2);o3(1:2);o4(1:2);o5(1:2);o6(1:2);o7(1:2);o9(1:2);
    o10(1:2);o11(1:2);o12(1:2);o13(1:2);o14(1:2);o15(1:2);o16(1:2);
    o17(1:2);o18(1:2);o19(1:2);o20(1:2);o21(1:2);o22(1:2)];
%% Define Map
% Creamos el mapa de 4x5 metros
map = binaryOccupancyMap(4,5,100);
% Se calcula la cantidad de objetos
%c_obs = length(Mobsc);
%xp = 2*ones(c_obs,1); yp = 2.5*ones(c_obs,1);

% Colocamos los obstaculos y se les suma su desfase
setOccupancy(map,Mobsc + [xp yp],ones(c_obs,1));
inflate(map,0.15);
mat = flip(double(occupancyMatrix(map)));

% Se genera la imagen del mapa con sus obstaculos
figure
show(map)
%% Generación de Trayectoria
start = robot7(1:2);    % Posición actual del robot en x y y
goal = [0,0];           % Meta a la que se desea llegar
%goal = [1, 1.6];
% Se genera la trayectoria con D*
ds = Dstar(mat);
ds.plan((goal + [2 2.5])*100);
ds.plot();
trayectoria1 = ds.query(round((start + [2 2.5])*100), 'animate');
tray = trayectoria1/100 - [2 2.5];
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
eO_D = 0;
eO_1 = 0;
EO = 0;

% Ciclo decontrol
k=1;
while(k<length(tray))
    xi = robotat_get_pose(robotat,7,'XYZ');
    x = xi(1); y = xi(2);  theta = (xi(6)+90)*pi/180;
    
    xg = tray(k,1);
    yg = tray(k,2);
    e = [xg - x; yg - y];
    thetag = atan2(e(2), e(1));

    eP = norm(e);
    eO = angdiff(theta,thetag);

    % Control de velocidad lineal
    kP = v0 * (1-exp(-alpha*eP^2)) / eP;
    v = kP*eP;

    % Control de velocidad angular
    eO_D = eO - eO_1;
    EO = EO + eO;
    w = kpO*eO + kiO*EO + kdO*eO_D;
    eO_1 = eO;

    % Se combinan los controladores
    u = [v; w];

    phi_R = (v+w*ell)/radio;
    phi_L = (v-w*ell)/radio;
    phi_L = convangvel(phi_L, 'rad/s', 'rpm');
    phi_R = convangvel(phi_R, 'rad/s', 'rpm');
    robotat_3pi_set_wheel_velocities(r7,phi_L,phi_R);
     k=k+1;
end
robotat_3pi_force_stop(r7);