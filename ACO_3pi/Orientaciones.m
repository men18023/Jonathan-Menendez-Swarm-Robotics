%% Obtener orientaciones para markers 1-10
%robotat = robotat_connect();
agents_zyx = robotat_get_pose(robotat,1:15,'eulzyx');
agents_quat = robotat_get_pose(robotat,1:15,'quat');
%save("Orientaciones",'agents_quat','agents_zyx')
robotat_trvisualize(robotat,1);

% z = (agents(6)+90)*pi/180;
% y = agents(5);
% x = agents(4);
% 
% phi4 = agents(4); 
% theta4 = agents(5);
% psi4 = agents(6);
% rot4 = rotz(phi4)*roty(theta4)*rotx(psi4);
% tras4 = [agents(1);agents(2);agents(3)];
% AT_4 = trans_hom(rot4,tras4);

% figure(2)
% trplot(AT_4,'frame','Obj4', 'color', "#EDB120",'arrows');

%%
load Orientaciones.mat
%markers_o = [data(:,1:3) data(:,7) data(:,4:6)];
bearing  = zeros(size(agents_quat,1),1);
bearing_deg  = zeros(size(agents_quat,1),1);

bearing1  = zeros(size(agents_zyx,1),1);
bearing_deg1  = zeros(size(agents_zyx,1),1);

conversiones = zeros(size(agents_quat,1),3);
for i = 1:size(agents_quat,1)
conversiones(i,:) = quat2eul(agents_quat(i,4:7),'ZYX'); 
bearing(i) = conversiones(i,1);
bearing_deg(i) = rad2deg(bearing(i));
end

for j = 1:size(agents_zyx,1)
%conversiones(j,:) = (agents_quat(j,4:6); 
bearing_deg1(j) = agents_zyx(j,4);
%bearing_deg1(j) = rad2deg(bearing1(j));
end