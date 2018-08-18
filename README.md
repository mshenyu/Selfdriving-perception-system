# Autonomous Vehicle System Building

## Lidar Configuration
![alt text](images/lidar.png "configuration")

## Sensor Calibration
### Multiple lidar calibration
During this project, there are two methods tried for multi-lidar calibration: GICP and Feature-based optimization.
#### 1. GICCP
Offered by PCL library. 

Result of GICP for 3 Velodynes:
![alt text](images/GICP.png "GICP")

#### 2. Feature-based optimization
This feature-based optimization is created by using the feature-corresponding optimization algorithm of [LOAM](http://www.roboticsproceedings.org/rss10/p07.pdf) developed by Ji Zhang and Sanjiv Singh. LOAM is developed for locating and mapping given the data of one Velodyne. And the result of the map is very desirable even with large  transformation and rotation. Since this feature-corresponding algorithm can be used to mapping two point clouds collected at two different time, this approach can also be used for multiple lidar calibration which has displacement in the perspective of space. 

Result of featured-based optimization:
![alt text](images/lm.png "feature-based optimization")

### Lidar camera calibration
For the lidar camera calibration, I follows the [publication](https://arxiv.org/abs/1705.09785) and [open code](https://github.com/ankitdhall/lidar_camera_calibration) of Ankit Dhall, Kunal Chelani, Vishnu Radhakrishnan and K.M. Krishna. They use aruco markers to find the 3D-to-3D correspondences of the the board’s edge and the marker’s center from lidar and camera respectively. Then they use UPnP to iterate 100 times to obtain the final result of the lidar-to-camera transformation matrix.
