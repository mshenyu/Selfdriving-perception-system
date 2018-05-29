#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace MSY_Denso{

class multi_lidar_transformation{


void setup(ros::NodeHandle);




private:
	ros::Subscriber sub_TL;
	ros::Subscriber sub_TR;
	ros::Publisher pub_TL;
	ros::Publisher pub_TC;
	ros::Publisher pub_TR;


}









}

#endif