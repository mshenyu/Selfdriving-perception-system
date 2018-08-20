function lidar_model(base_center,base_width,base_length,base_height,lidar1_pos,lidar2_pos,lidar3_pos,H2,H3,roll2,roll3)
D2R = pi/180;
%base
d=[-1 1];
[x,y,z]=meshgrid(d,d,d);
points=[x(:),y(:),z(:);zeros(1,3)];
points(:,1) = points(:,1).*base_length+base_center(1,1);
points(:,2) = points(:,2).*base_width+base_center(2,1);
points(:,3) = points(:,3).*base_height+base_center(3,1);

% points=points+0.001*rand(size(points));
T1=delaunayn(points);
tetramesh(T1,points,'FaceColor',[1 0.5 0],...
    'EdgeColor',[1 0.5 0],'FaceAlpha',0.3);

hold on

r = 0.1;
h = 0.3;
% %lidar1
% [X,Y,Z] = cylinder(r);
% Z = Z * h;
% X = X + lidar1_pos(1,1);
% Y = Y + lidar1_pos(2,1);
% Z = Z + lidar1_pos(3,1);
% 
% surf(X,Y,Z)
% hold on
% %lidar2
% [X,Y,Z] = cylinder(r);
% Z = Z * h;
% X = X + lidar2_pos(1,1);
% Y = Y + lidar2_pos(2,1);
% Z = Z + lidar2_pos(3,1);
% 
% 
% rotate(surf(X,Y,Z),[1,0,0],roll2/D2R)
% 
% hold on
% %lidar3
% [X,Y,Z] = cylinder(r);
% Z = Z * h;
% X = X + lidar3_pos(1,1);
% Y = Y + lidar3_pos(2,1);
% Z = Z + lidar3_pos(3,1);
% 
% rotate(surf(X,Y,Z),[1,0,0],roll3/D2R)
% 
% hold on

end