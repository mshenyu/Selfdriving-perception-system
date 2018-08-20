function [xyzPoints1,xyzPoints2,xyzPoints3] = generate_scan_points( ...
channel_number,scan_number,L_FOV_bound,vel_resolution,range,H1,H2,H3, ...
lidar1_pos,lidar2_pos,lidar3_pos,car_center,car_width,car_length,base_center,base_width,base_length,side_range)

D2R = pi/180;
xyzPoints = zeros(channel_number,scan_number,3);
xyzPoints1 = xyzPoints;
xyzPoints2 = xyzPoints;
xyzPoints3 = xyzPoints;

half_FOV_length = sqrt(range^2-side_range^2);
d_horizon2 = abs(lidar2_pos(2)-lidar1_pos(2));
d_horizon3 = abs(lidar3_pos(2)-lidar1_pos(2));
left_range1 = side_range;
right_range1 = side_range;
left_range2 = side_range-d_horizon2;
right_range2 = side_range+d_horizon2;
left_range3 = side_range+d_horizon3;
right_range3 = side_range-d_horizon3;
down_range1 = lidar1_pos(3);
down_range2 = lidar2_pos(3);
down_range3 = lidar3_pos(3);
%FOV
    %xy all are positive
front_angle_L1 = atan2(side_range,half_FOV_length);
front_angle_R1 = atan2(side_range,half_FOV_length);

front_angle_L2 = atan2(left_range2,half_FOV_length);
front_angle_R2 = atan2(right_range2,half_FOV_length);

front_angle_L3 = atan2(left_range3,half_FOV_length);
front_angle_R3 = atan2(right_range3,half_FOV_length);

%generate scan points
for k = 1:channel_number
    alpha = L_FOV_bound + (k-1) * vel_resolution;
    horizontal_range = range*cos(alpha);
    vertical_pose = range*sin(alpha);
    for i=1:scan_number
        theta = 2*pi*i/scan_number;
        xyzPoints(k,i,:) = [horizontal_range*cos(theta),horizontal_range*sin(theta),vertical_pose]; 
        
% Lidar1 
        homo_pos = H1*[xyzPoints(k,i,1);xyzPoints(k,i,2);xyzPoints(k,i,3);1];
        %??
%         ray_horizon_dir = homo_pos - [lidar1_pos(1,1);lidar1_pos(2,1);homo_pos(3,1);1];
        ray_horizon_dir = homo_pos - [lidar1_pos;1];
        beta = atan2(homo_pos(3)-lidar1_pos(3),sqrt(ray_horizon_dir(1)^2+ray_horizon_dir(2)^2));
        horizontal_angle = atan2(ray_horizon_dir(2),ray_horizon_dir(1));
        rat_base = (lidar1_pos(3) - base_center(3))/abs(ray_horizon_dir(3));
        x_base = rat_base*ray_horizon_dir(1)+lidar1_pos(1)-lidar1_pos(1);
        y_base = rat_base*ray_horizon_dir(2)+lidar1_pos(2)-lidar1_pos(2);
        rat_car = (lidar1_pos(3) - car_center(3))/abs(ray_horizon_dir(3));
        x_car = rat_car*ray_horizon_dir(1)+lidar1_pos(1)-lidar1_pos(1);
        y_car = rat_car*ray_horizon_dir(2)+lidar1_pos(2)-lidar1_pos(2);
        if x_base>-base_length && x_base<base_length && y_base>-base_width && y_base<base_width && ray_horizon_dir(3)<0
            xyzPoints1(k,i,:) =[x_base;y_base;base_center(3)];

        elseif x_car>-car_length && x_car<car_length && y_car>-car_width && y_car<car_width && ray_horizon_dir(3)<0
            xyzPoints1(k,i,:) =[x_car;y_car;car_center(3)];

        elseif horizontal_angle >= front_angle_L1 && horizontal_angle <= pi - front_angle_L1 %left
            x = left_range1/tan(horizontal_angle);
            y = side_range;
            range_unify = abs(left_range1/sin(horizontal_angle));
            z = lidar1_pos(3)+ range_unify *tan(beta);
            xyzPoints1(k,i,:) = [x;y;z];
        elseif horizontal_angle >= -pi + front_angle_R1 && horizontal_angle <= -front_angle_R1 %right
            x = right_range1/tan(-horizontal_angle);
            y = -side_range;
            range_unify = abs(right_range1/sin(horizontal_angle));
            z = lidar1_pos(3)+ range_unify *tan(beta);
            xyzPoints1(k,i,:) = [x;y;z];
        else
            z_unify = lidar1_pos(3)+range*tan(beta);
            xyzPoints1(k,i,:) = [range*cos(horizontal_angle);range*sin(horizontal_angle);z_unify];
        end
        if xyzPoints1(k,i,3)<0
            x = (lidar1_pos(3)/tan(beta))*cos(horizontal_angle);
            y = (lidar1_pos(3)/tan(beta))*sin(horizontal_angle);
            z = 0;
            xyzPoints1(k,i,:) =[x;y;z];
        end
% Lidar2
        homo_pos = H2*[xyzPoints(k,i,1);xyzPoints(k,i,2);xyzPoints(k,i,3);1];
        ray_horizon_dir = homo_pos - [lidar2_pos;1];
        beta = atan2(ray_horizon_dir(3),sqrt(ray_horizon_dir(1)^2+ray_horizon_dir(2)^2));
        horizontal_angle = atan2(ray_horizon_dir(2),ray_horizon_dir(1));
        rat_base = (lidar2_pos(3) - base_center(3))/abs(ray_horizon_dir(3));
        x_base = rat_base*ray_horizon_dir(1)+lidar2_pos(1)-lidar1_pos(1);
        y_base = rat_base*ray_horizon_dir(2)+lidar2_pos(2)-lidar1_pos(2);
        rat_car = (lidar2_pos(3) - car_center(3))/abs(ray_horizon_dir(3));
        x_car = rat_car*ray_horizon_dir(1)+lidar2_pos(1)-lidar1_pos(1);
        y_car = rat_car*ray_horizon_dir(2)+lidar2_pos(2)-lidar1_pos(2);
        if x_base>-base_length && x_base<base_length && y_base>-base_width && y_base<base_width && ray_horizon_dir(3)<0
            xyzPoints2(k,i,:) =[x_base;y_base;base_center(3)];

        elseif x_car>-car_length && x_car<car_length && y_car>-car_width && y_car<car_width && ray_horizon_dir(3)<0
            xyzPoints2(k,i,:) =[x_car;y_car;car_center(3)];

        elseif horizontal_angle >= front_angle_L2 && horizontal_angle <= pi - front_angle_L2 %left
            x = left_range2/tan(horizontal_angle);
            y = side_range;
            range_unify = abs(left_range2/sin(horizontal_angle));
            z = lidar2_pos(3)+ range_unify *tan(beta);
            xyzPoints2(k,i,:) = [x;y;z];
        elseif horizontal_angle >= -pi + front_angle_R2 && horizontal_angle <= -front_angle_R2 %right
            x = right_range2/tan(-horizontal_angle);
            y = -side_range;
            range_unify = abs(right_range2/sin(horizontal_angle));
            z = lidar2_pos(3)+ range_unify *tan(beta);
            xyzPoints2(k,i,:) = [x;y;z];

        else
            alpha_unify = atan2(ray_horizon_dir(2),ray_horizon_dir(1));
            range_unify = compute_unified_range(range,d_horizon2,alpha_unify+pi/2);
            z_unify = lidar2_pos(3)+range_unify*tan(beta);
            x = range_unify*cos(alpha_unify)+lidar2_pos(1)-lidar1_pos(1);
            y = range_unify*sin(alpha_unify)+lidar2_pos(2)-lidar1_pos(2);
            xyzPoints2(k,i,:) = [x;y;z_unify];
        end
        if xyzPoints2(k,i,3)<0
            x = (lidar2_pos(3)/tan(-beta))*cos(horizontal_angle)+lidar2_pos(1)-lidar1_pos(1);
            y = (lidar2_pos(3)/tan(-beta))*sin(horizontal_angle)+lidar2_pos(2)-lidar1_pos(2);%???????????????
            z = 0;
            xyzPoints2(k,i,:) =[x;y;z];
        end

% Lidar3
        homo_pos = H3*[xyzPoints(k,i,1);xyzPoints(k,i,2);xyzPoints(k,i,3);1];
        ray_horizon_dir = homo_pos - [lidar3_pos;1];
        beta = atan2(homo_pos(3)-lidar3_pos(3),sqrt(ray_horizon_dir(1)^2+ray_horizon_dir(2)^2));
        horizontal_angle = atan2(ray_horizon_dir(2),ray_horizon_dir(1));
        rat_base = (lidar3_pos(3) - base_center(3))/abs(ray_horizon_dir(3));
        x_base = rat_base*ray_horizon_dir(1)+lidar3_pos(1)-lidar1_pos(1);
        y_base = rat_base*ray_horizon_dir(2)+lidar3_pos(2)-lidar1_pos(2);
        rat_car = (lidar3_pos(3) - car_center(3))/abs(ray_horizon_dir(3));
        x_car = rat_car*ray_horizon_dir(1)+lidar3_pos(1)-lidar1_pos(1);
        y_car = rat_car*ray_horizon_dir(2)+lidar3_pos(2)-lidar1_pos(2);
        if x_base>-base_length && x_base<base_length && y_base>-base_width && y_base<base_width && ray_horizon_dir(3)<0
            xyzPoints3(k,i,:) =[x_base;y_base;base_center(3)];

        elseif x_car>-car_length && x_car<car_length && y_car>-car_width && y_car<car_width && ray_horizon_dir(3)<0
            xyzPoints3(k,i,:) =[x_car;y_car;car_center(3)];

        elseif horizontal_angle >= front_angle_L3 && horizontal_angle <= pi - front_angle_L3 %left
            x = left_range3/tan(horizontal_angle);
            y = side_range;
            range_unify = abs(left_range3/sin(horizontal_angle));
            z = lidar3_pos(3)+ range_unify *tan(beta);
            xyzPoints3(k,i,:) = [x;y;z];
        elseif horizontal_angle >= -pi + front_angle_R3 && horizontal_angle <= -front_angle_R3 %right
            x = right_range3/tan(-horizontal_angle);
            y = -side_range;
            range_unify = abs(right_range3/sin(horizontal_angle));
            z = lidar3_pos(3)+ range_unify *tan(beta);
            xyzPoints3(k,i,:) = [x;y;z];

        else
            alpha_unify = atan2(ray_horizon_dir(2),ray_horizon_dir(1));
            range_unify = compute_unified_range(range,d_horizon3,-alpha_unify+pi/2);
            z_unify = lidar3_pos(3)+range_unify*tan(beta);
            x = range_unify*cos(alpha_unify)+lidar3_pos(1)-lidar1_pos(1);
            y = range_unify*sin(alpha_unify)+lidar3_pos(2)-lidar1_pos(2);
            xyzPoints3(k,i,:) = [x;y;z_unify];
        end
        if xyzPoints3(k,i,3)<0
            x = (lidar3_pos(3)/tan(-beta))*cos(horizontal_angle)+lidar3_pos(1)-lidar1_pos(1);
            y = (lidar3_pos(3)/tan(-beta))*sin(horizontal_angle)+lidar3_pos(2)-lidar1_pos(2);%???????????????
            z = 0;
            xyzPoints3(k,i,:) =[x;y;z];
        end
        
        
        if xyzPoints1(k,i,3)>5
            xyzPoints1(k,i,:)=[0;0;0];
        end
        if xyzPoints2(k,i,3)>5
            xyzPoints2(k,i,:)=[0;0;0];
        end
        if xyzPoints3(k,i,3)>5
            xyzPoints3(k,i,:)=[0;0;0];
        end
        
    end
end




%???????????????? showcase?information flow? weight integ? bulletin