#ifndef MULTI_LIDAR_CALIB_H
#define MULTI_LIDAR_CALIB_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/registration/icp.h>
// #include <pcl/registration/gicp.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/impl/pcl_base.hpp>
// #include <pcl/kdtree/impl/kdtree_flann.hpp>
// #include <pcl/search/impl/kdtree.hpp>
// #include <pcl/search/impl/organized.hpp>
// #include <pcl/features/impl/normal_3d.hpp>
// #include <pcl/filters/filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


pcl::PointCloud<pcl::PointXYZI> CloudFR;
pcl::PointCloud<pcl::PointXYZI> CloudFL;
pcl::PointCloud<pcl::PointXYZI> CloudTL;
pcl::PointCloud<pcl::PointXYZI> CloudTC;
pcl::PointCloud<pcl::PointXYZI> CloudTR;

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
ros::Publisher pub5;


#endif