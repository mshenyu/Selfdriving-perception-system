function [xyzPoints1,xyzPoints2,xyzPoints3] = generate_scan_points( ...
channel_number,scan_number,L_FOV_bound,vel_resolution,range,H1,H2,H3, ...
lidar1_pos,lidar2_pos,lidar3_pos,side_range)

D2R = pi/180;
xyzPoints = zeros(channel_number,scan_number,3);
xyzPoints1 = xyzPoints;
xyzPoints2 = xyzPoints;
xyzPoints3 = xyzPoints;

half_FOV_length = sqrt(range^2-side_range^2);
d_horizon2 = abs(lidar2_pos(3)-lidar1_pos(3));
d_horizon3 = abs(lidar3_pos(3)-lidar1_pos(3));
left_range2 = side_range-d_horizon2;
right_range2 = side_range+d_horizon2;
%FOV
front_angle_L1 = atan2(side_range,half_FOV_length);
front_angle_R1 = atan2(side_range,half_FOV_length);

front_angle_L2 = atan2(left_range2,half_FOV_length);
front_angle_R2 = atan2(right_range2,half_FOV_length);

front_angle_L3 = atan2((side_range+d_horizon3),half_FOV_length);
front_angle_R3 = atan2((side_range-d_horizon3),half_FOV_length);


%generate scan points
for k = 1:channel_number
    alpha = L_FOV_bound + (k-1) * vel_resolution;
    horizontal_range = range*cos(alpha);
    vertical_pose = range*sin(alpha);
    for i=1:scan_number
        theta = 360*i/scan_number;
        xyzPoints(k,i,:) = [horizontal_range*cos(theta),horizontal_range*sin(theta),vertical_pose]; 
        
% Lidar1        
        homo_pos = H1*[xyzPoints(k,i,1);xyzPoints(k,i,2);xyzPoints(k,i,3);1];
        %extension
        ray_horizon_dir = homo_pos - [lidar1_pos(1,1);lidar1_pos(2,1);homo_pos(3,1);1];
        beta = atan2(homo_pos(3)-lidar1_pos(3),sqrt(ray_horizon_dir(1)^2+ray_horizon_dir(2)^2));
        z_unify = lidar1_pos(3)+range*tan(beta);
        alpha_unify = atan2(ray_horizon_dir(2),ray_horizon_dir(1));
%         d_horizon = abs(lidar1_pos(3)-lidar1_pos(3));
%         range_unify = compute_unified_range(range,d_horizon,alpha_unify);
        xyzPoints1(k,i,:) = [range*cos(alpha_unify);range*sin(alpha_unify);z_unify];

% Lidar2
        homo_pos = H2*[xyzPoints(k,i,1);xyzPoints(k,i,2);xyzPoints(k,i,3);1];
        %extension
        ray_horizon_dir = homo_pos - [lidar2_pos(1);lidar2_pos(2);homo_pos(3);1];
            %horizontal_angle
        horizontal_angle = atan2(ray_horizon_dir(2),ray_horizon_dir(1));
%         horizontal_angle/D2R
        beta = atan2(homo_pos(3)-lidar2_pos(3),sqrt(ray_horizon_dir(1)^2+ray_horizon_dir(2)^2));
        if horizontal_angle >= front_angle_L2 && horizontal_angle <= pi - front_angle_L2 %left
            x = left_range2/tan(horizontal_angle);
            y = side_range;
            range_unify = abs(left_range2/sin(horizontal_angle));
            z = lidar2_pos(3)+ range_unify *tan(beta);
%             xyzPoints2(k,i,:) = [x;y;z];
            xyzPoints2(k,i,:) = [0;0;0];
%             alpha_origin = 
        elseif horizontal_angle >= -pi - front_angle_R2 && horizontal_angle <= front_angle_R2 %right
            x = right_range2/tan(-horizontal_angle);
            y = -side_range;
            range_unify = abs(right_range2/sin(horizontal_angle));
            z = lidar2_pos(3)+ range_unify *tan(beta);
%             xyzPoints2(k,i,:) = [x;y;z];
            xyzPoints2(k,i,:) = [0;0;0];
        else
            alpha_unify = pi - atan2(ray_horizon_dir(1),ray_horizon_dir(2));
            range_unify = compute_unified_range(range,d_horizon2,alpha_unify);
            z_unify = lidar2_pos(3)+range_unify*tan(beta);
            alpha_origin = pi - acos((range^2-d_horizon2^2+range_unify^2)/(2*range*range_unify)) - alpha_unify;
            xyzPoints2(k,i,:) = [range*cos(pi/2-alpha_origin);range*sin(pi/2-alpha_origin);z_unify];
        end
% Lidar3
        homo_pos = H3*[xyzPoints(k,i,1);xyzPoints(k,i,2);xyzPoints(k,i,3);1];
        %extension
        ray_horizon_dir = homo_pos - [lidar3_pos(1);lidar3_pos(2);homo_pos(3);1];
        beta = atan2(homo_pos(3)-lidar3_pos(3),sqrt(ray_horizon_dir(1)^2+ray_horizon_dir(2)^2));
        
        alpha_unify = pi - atan2(ray_horizon_dir(1),ray_horizon_dir(2));
        
        range_unify = compute_unified_range(range,d_horizon3,alpha_unify);
        z_unify = lidar3_pos(3)+range_unify*tan(beta);
        alpha_origin = pi - acos((range^2-d_horizon3^2+range_unify^2)/(2*range*range_unify)) - alpha_unify;
        xyzPoints3(k,i,:) = [range*cos(pi/2-alpha_origin);range*sin(pi/2-alpha_origin);z_unify];


       
        
    end
end



% %generate scan points
% for k = 1:channel_number
%     alpha = L_FOV_bound + (k-1) * vel_resolution;
%     horizontal_range = range*cos(alpha);
%     vertical_pose = range*sin(alpha);
%     for i=1:scan_number
%         theta = 360*i/scan_number;
%         xyzPoints(k,i,:) = [horizontal_range*cos(theta),horizontal_range*sin(theta),vertical_pose];
%         homo_pos = H2*[xyzPoints(k,i,1);xyzPoints(k,i,2);xyzPoints(k,i,3);1];
%         xyzPoints2(k,i,:) = homo_pos(1:3);
%     end
% 
% end
% 
% 
% %generate scan points
% for k = 1:channel_number
%     alpha = L_FOV_bound + (k-1) * vel_resolution;
%     horizontal_range = range*cos(alpha);
%     vertical_pose = range*sin(alpha);
%     for i=1:scan_number
%         theta = 360*i/scan_number;
%         xyzPoints(k,i,:) = [horizontal_range*cos(theta),horizontal_range*sin(theta),vertical_pose];
%         homo_pos = H3*[xyzPoints(k,i,1);xyzPoints(k,i,2);xyzPoints(k,i,3);1];
%         xyzPoints3(k,i,:) = homo_pos(1:3);
%     end
% 
% end


%???????????????? showcase?information flow? weight integ? bulletin