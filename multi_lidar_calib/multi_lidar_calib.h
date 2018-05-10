#ifndef MULTI_LIDAR_CALIB_H
#define MULTI_LIDAR_CALIB_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/filters/filter.h>

class msy_multi_lidar{
public:

void lidar_read1( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg);
void lidar_read3( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg);
void lidar_read4( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg);
void lidar_read5( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg);

void setup(ros::NodeHandle& nh);
void spin();
bool process();


	pcl::PointCloud<pcl::PointXYZI>::Ptr CloudFR_pt;
	pcl::PointCloud<pcl::PointXYZI>::Ptr CloudTF_pt;
	pcl::PointCloud<pcl::PointXYZI>::Ptr CloudTC_pt;
	pcl::PointCloud<pcl::PointXYZI>::Ptr CloudTR_pt;

	pcl::PointCloud<pcl::PointXYZI> CloudFR;
	pcl::PointCloud<pcl::PointXYZI> CloudTF;
	pcl::PointCloud<pcl::PointXYZI> CloudTC;
	pcl::PointCloud<pcl::PointXYZI> CloudTR;

private:
	ros::Subscriber sub1;
	ros::Publisher pub1;
	ros::Subscriber sub3;
	ros::Publisher pub3;
	ros::Subscriber sub4;
	ros::Publisher pub4;
	ros::Subscriber sub5;
	ros::Publisher pub5;





};

void msy_multi_lidar::setup(ros::NodeHandle& nh){
sub1 = nh.subscribe("ns1/velodyne_points", 10, &msy_multi_lidar::lidar_read1, this);
pub1 = nh.advertise<sensor_msgs::PointCloud2>("/msy_FR", 10);
sub3 = nh.subscribe("ns3/velodyne_points", 10, &msy_multi_lidar::lidar_read3, this);
pub3 = nh.advertise<sensor_msgs::PointCloud2>("/msy_TF", 10);
sub4 = nh.subscribe("ns4/velodyne_points", 10, &msy_multi_lidar::lidar_read4, this);
pub4 = nh.advertise<sensor_msgs::PointCloud2>("/msy_TC", 10);
sub5 = nh.subscribe("ns5/velodyne_points", 10, &msy_multi_lidar::lidar_read5, this);
pub5 = nh.advertise<sensor_msgs::PointCloud2>("/msy_TR", 10);

}

void msy_multi_lidar::spin(){
	ros::Rate rate(100);
	ros::spinOnce();
	rate.sleep();
	// loop until shutdown
	while (process() && ros::ok()) {
	ros::spinOnce();
	rate.sleep();
	}
}



#endif
