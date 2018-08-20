clear
D2R = pi/180;
% blue1, green2, purple3
%flag
even_flag = 0;

%paramter
range = 30;
depth_range = 1;
depth_resolution = 1;
scan_number = 2187;         %700000/10/32?0.16 horizontal resolution
channel_number = 32;
vel_resolution = 1.3 * D2R;%radius
L_FOV_bound = -30 * D2R;%radius
    %road
lame_width = 3.6;
lame_number = 2;
road_width = lame_width*lame_number;
side_range = lame_width*(lame_number-0.5);
road_height = 4;
    %base
base_width = 0.4445;
base_length = 0.193675;
base_height = 0.025;
base_center = [0;0;1.5367];

car_width = 1.58242/2;
car_length = 2.32; % 0.711;
car_height = 0.025;
car_center = [0;0;0.889];%1.5367];

neighbor_range = (range*tan(10 *D2R)+range*tan(30 *D2R))/31*1.05;

h1 = 0.2545;
h2 = 0.1624;
h3 = 0.1624;
% h1 = 0.4;
% h2 = 0.1;
% h3 = 0.1;

ytrans = 0.4445;
lidar1_pos = [0;0;1.5621+h1];
lidar2_pos = [0;+ytrans;1.5621+h2]; %left
lidar3_pos = [0;-ytrans;1.5621+h3];  %right

for_tune = 18.981891712439538;% 15.8655;
roll2 = -for_tune  *D2R;
pitch2 = 0  *D2R;%1.3/3
yaw2 = 0  *D2R;
roll3 = for_tune  *D2R;
pitch3 = 0  *D2R;%1.3*2/3
yaw3 = 0  *D2R;
%Transform of lidar2
T1 = lidar1_pos;
H1 = [eye(3,3) T1;
     [0 0 0 ] 1];

%Transform of lidar2
Rx2 = [1           0              0;
      0     cos(roll2) -sin(roll2);
      0     sin(roll2)  cos(roll2)];

Ry2 = [cos(pitch2) 0 sin(pitch2);
      0           1          0;
      -sin(pitch2) 0 cos(pitch2)];

Rz2 = [cos(yaw2) -sin(yaw2)      0;
      sin(yaw2)  cos(yaw2)      0;
      0          0              1];
R2 = Rx2*Ry2*Rz2;
T2 = lidar2_pos;
H2 = [R2 T2;
     [0 0 0] 1];

%Transform of lidar3
Rx3 = [1           0              0;
      0     cos(roll3) -sin(roll3);
      0     sin(roll3)  cos(roll3)];

Ry3 = [cos(pitch3) 0 sin(pitch3);
      0           1          0;
      -sin(pitch3) 0 cos(pitch3)];

Rz3 = [cos(yaw3) -sin(yaw3)      0;
      sin(yaw3)  cos(yaw3)      0;
      0          0              1];
R3 = Rx3*Ry3*Rz3;
T3 = lidar3_pos;
H3 = [R3 T3;
     [0 0 0] 1];
 

xyz_even_test = [];

tic;
%%%%%%%%%%%%%%%%% generate scan points %%%%%%%%%%%%%%%%% 
depth = fix(depth_range/depth_resolution);
color1 = zeros(channel_number,scan_number,3);
color2 = color1;
color3 = color1;
color1(:,:,1) = 0;
color1(:,:,2) = 0;
color1(:,:,3) = 1;

color2(:,:,1) = 24/255;
color2(:,:,2) = 139/255;
color2(:,:,3) = 24/255;

color3(:,:,1) = 255/255;
color3(:,:,2) = 10/255;
color3(:,:,3) = 220/255;
figure;
for d=1:depth
    distance = range - depth_range/2 + d*depth_resolution;

    [xyzPoints1,xyzPoints2,xyzPoints3] = generate_scan_points(channel_number,scan_number,L_FOV_bound,vel_resolution,distance,H1,H2,H3,lidar1_pos,lidar2_pos,lidar3_pos,car_center,car_width,car_length,base_center,base_width,base_length,side_range);

%     for k = 1:channel_number
%         for i=1:scan_number
%             if xyzPoints1(k,i,3)>5
%                 xyzPoints1(k,i,:) = [0 0 0];
%             else
%                 xyz_even_test = [xyz_even_test;xyzPoints1(k,i,:)];
%             end
%             
%             if xyzPoints2(k,i,3)>5
%                 xyzPoints2(k,i,:) = [0 0 0];
%             else
%                 xyz_even_test = [xyz_even_test;xyzPoints2(k,i,:)];
%             end
%             if xyzPoints3(k,i,3)>5
%                 xyzPoints3(k,i,:) = [0 0 0];
%             else
%                 xyz_even_test = [xyz_even_test;xyzPoints3(k,i,:)];
%             end
% 
%         end
%     end
    
    ptCloud1 = pointCloud(xyzPoints1(:,:,:),'c',color1);
    ptCloud2 = pointCloud(xyzPoints2(:,:,:),'c',color2);
    ptCloud3 = pointCloud(xyzPoints3(:,:,:),'c',color3);


    color_even = zeros(size(xyz_even_test));
    color_even(:,1) = 255/255;
    color_even(:,2) = 10/255;
    color_even(:,3) = 220/255;
%     ptCloud_test = pointCloud(xyz_even_test(:,:));%,'c',color_even);

%     
    pcshow(ptCloud1,'MarkerSize',100);
    hold on
    pcshow(ptCloud2,'MarkerSize',100);
    hold on
    pcshow(ptCloud3,'MarkerSize',100);
    hold on
%     pcshow(ptCloud_test,'MarkerSize',10);
%     hold on

end

% compute evenness
if even_flag == 1
    evenness = 0;
    for n = 1:size(xyz_even_test,1)
        cnt = 0;
        dist_var = 0;
        dist_mean = 0;
        neighbors = [];
        for k = 1:size(xyz_even_test,1)
            if xyz_even_test(k,1)>xyz_even_test(n,1)-neighbor_range && xyz_even_test(k,1)<xyz_even_test(n,1)+neighbor_range ...
               && xyz_even_test(k,2)>xyz_even_test(n,2)-neighbor_range && xyz_even_test(k,2)<xyz_even_test(n,2)+neighbor_range ...
               && xyz_even_test(k,3)>xyz_even_test(n,3)-neighbor_range && xyz_even_test(k,3)<xyz_even_test(n,3)+neighbor_range ...
               && n ~= k
                neighbors = [neighbors;xyz_even_test(k,:)];
                dist_mean = dist_mean + sqrt(norm(xyz_even_test(n,:) - xyz_even_test(k,:),2));
                cnt = cnt+1;
            end
        end
        if cnt == 0
            dist_var = 0;
        else
            dist_mean = dist_mean/cnt;
            for c = 1:cnt
                dist_var = dist_var + (sqrt(norm(xyz_even_test(n,:)-neighbors(c,:),2))-dist_mean)^2;
            end
            dist_var = dist_var/cnt;
        end
        evenness = evenness + dist_var;
    end
    
    format long
    evenness = sqrt(evenness/n)
end

lidar_model(car_center,car_width,car_length,car_height,lidar1_pos,lidar2_pos,lidar3_pos,H2,H3,roll2,roll3);
lidar_model(base_center,base_width,base_length,base_height,lidar1_pos,lidar2_pos,lidar3_pos,H2,H3,roll2,roll3);
title('range:120m * 18m * 4m');
% title('roof');
xlabel('X');j
ylabel('Y');
zlabel('Z');

toc;

%lidar extend
%methodology: how to define the object function and constreints
%case
%number
%#2 #3 compare  64-channel
%sparsity
%weights distribution range, car and people
%lidar reconstruction paper
%ISTC
%MIT:
%Stanfod cars: 
%Best paper in ISTC
