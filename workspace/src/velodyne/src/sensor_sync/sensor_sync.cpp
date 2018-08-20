#include <math.h> 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_conversions/pcl_conversions.h>

using namespace	std;
ros::Publisher pub_lidar1;
ros::Publisher pub_lidar2;
ros::Publisher pub_lidar3;
pcl::PointCloud<pcl::PointXYZI> Cloud_TL;
pcl::PointCloud<pcl::PointXYZI> Cloud_TC;
pcl::PointCloud<pcl::PointXYZI> Cloud_TR;
double gps_vel;
double scan_period = 0.1;
#define PI float(M_PI)

void gps( const geometry_msgs::TwistWithCovarianceStamped & msg_vel){

//TODO: calibrate GPS

  gps_vel = sqrt(msg_vel.twist.twist.linear.x*msg_vel.twist.twist.linear.x + msg_vel.twist.twist.linear.y*msg_vel.twist.twist.linear.y + msg_vel.twist.twist.linear.z*msg_vel.twist.twist.linear.z);
}

// void callback_lidar_sync(const sensor_msgs::PointCloud2ConstPtr& cloud_msg_1, 
//                          const sensor_msgs::PointCloud2ConstPtr& cloud_msg_2, 
//                          const sensor_msgs::PointCloud2ConstPtr& cloud_msg_3,
//                          const sensor_msgs::ImageConstPtr& image_msg){
void callback_lidar_sync(const sensor_msgs::PointCloud2ConstPtr& cloud_msg_1, 
                         const sensor_msgs::PointCloud2ConstPtr& cloud_msg_2, 
                         const sensor_msgs::PointCloud2ConstPtr& cloud_msg_3){
	cout << "callback_lidar_sync" << endl;
    pcl::fromROSMsg(*cloud_msg_1, Cloud_TL);
    pcl::fromROSMsg(*cloud_msg_2, Cloud_TC);
    pcl::fromROSMsg(*cloud_msg_3, Cloud_TR);

    // gps_vel = 0;
    double displaceTL = gps_vel*(cloud_msg_2->header.stamp.toSec() - cloud_msg_1->header.stamp.toSec());
    double displaceTR = gps_vel*(cloud_msg_2->header.stamp.toSec() - cloud_msg_3->header.stamp.toSec());

    cout << "velocity: " << gps_vel << "			time difference: " << (cloud_msg_2->header.stamp.toSec() - cloud_msg_3->header.stamp.toSec()) << endl;

	float startOri;
	float endOri;

	//Sync TL**************************
	size_t cloudSizeTL = Cloud_TL.points.size();

  	startOri = -std::atan2(Cloud_TL[0].y, Cloud_TL[0].x);
  	endOri = -std::atan2(Cloud_TL[cloudSizeTL - 1].y, Cloud_TL[cloudSizeTL - 1].x);

  	if(startOri<0) startOri += 2*PI;
  	if(endOri<0) endOri += 2*PI;

	for (size_t i = 0; i < cloudSizeTL; i++) {
		pcl::PointXYZI& point = Cloud_TL.points[i];
		double mm = point.y;
		double scan_angle = -atan2(point.y, point.x);
		if(scan_angle<0) scan_angle += 2*PI;
		if(startOri<endOri){
			if(endOri - startOri > PI){
				point.y += gps_vel*scan_period*(scan_angle - startOri)/(endOri - startOri);
				// cout << "endOri - startOri >= PI" << endl;
			}
			else{
				// cout << "endOri - startOri < PI" << endl;
				if(i < cloudSizeTL/2){	
					point.y += gps_vel*scan_period*(scan_angle - startOri + ((scan_angle>=startOri)?0:(2*PI)) )/(endOri - startOri + 2*PI);
					// cout << "			1111111111111111111111111:   " << point.y - mm << endl;//he
				}
				else {
					point.y += gps_vel*scan_period*(scan_angle - startOri + ((scan_angle < endOri)?2*PI:0) )/(endOri - startOri + 2*PI);
					// cout << "			222222222222222:   " << point.y - mm << endl;
				}
			}
		} 
		else {
			if (startOri - endOri < PI){
				point.y += gps_vel*scan_period*( ((scan_angle<=endOri)?(scan_angle+2*PI):scan_angle) - startOri)/(endOri+2*PI - startOri);
				// cout << "startOri - endOri < PI" << endl;
			}
			else{
				// cout << "startOri - endOri >= PI" << endl;
				if(i < cloudSizeTL/2){
					// cout << "			1111111111111111111111111" << endl;
					point.y += gps_vel*scan_period*( scan_angle - startOri + ((scan_angle>=startOri)?0:(2*PI)) )/(endOri + 4*PI - startOri);
				}
				else{
					// cout << "			222222222222222" << endl;
					point.y += gps_vel*scan_period*( scan_angle - startOri + 2*PI + ((scan_angle<endOri)?(2*PI):0) )/(endOri + 4*PI - startOri);
				}
			}
		}  
		point.y -= displaceTL;
	}

	//Sync TC**************************
	size_t cloudSizeTC = Cloud_TC.points.size();

  	startOri = -std::atan2(Cloud_TC[0].y, Cloud_TC[0].x);
  	endOri = -std::atan2(Cloud_TC[cloudSizeTC - 1].y, Cloud_TC[cloudSizeTC - 1].x);


  	if(startOri<0) startOri += 2*PI;
  	if(endOri<0) endOri += 2*PI;

  	cout << "startOri: " << startOri*180/PI << "	endOri: " << endOri*180/PI << endl;
	for (size_t i = 0; i < cloudSizeTC; i++) {
		pcl::PointXYZI& point = Cloud_TC.points[i];
		double mm = point.y;
		double scan_angle = -atan2(point.y, point.x);
		if(scan_angle<0) scan_angle += 2*PI;
		if(startOri<endOri){
			if(endOri - startOri > PI){
				point.y += gps_vel*scan_period*(scan_angle - startOri)/(endOri - startOri);
				// cout << "endOri - startOri >= PI" << endl;
			}
			else{
				// cout << "endOri - startOri < PI" << endl;
				if(i < cloudSizeTC/2){	
					point.y += gps_vel*scan_period*(scan_angle - startOri + ((scan_angle>=startOri)?0:(2*PI)) )/(endOri - startOri + 2*PI);
					// cout << "			1111111111111111111111111:   " << point.y - mm << endl;//he
				}
				else {
					point.y += gps_vel*scan_period*(scan_angle - startOri + ((scan_angle < endOri)?2*PI:0) )/(endOri - startOri + 2*PI);
					// cout << "			222222222222222:   " << point.y - mm << endl;
				}
			}
		} 
		else {
			if (startOri - endOri < PI){
				point.y += gps_vel*scan_period*( ((scan_angle<=endOri)?(scan_angle+2*PI):scan_angle) - startOri)/(endOri+2*PI - startOri);
				// cout << "startOri - endOri < PI" << endl;
			}
			else{
				// cout << "startOri - endOri >= PI" << endl;
				if(i < cloudSizeTC/2){
					// cout << "			1111111111111111111111111" << endl;
					point.y += gps_vel*scan_period*( scan_angle - startOri + ((scan_angle>=startOri)?0:(2*PI)) )/(endOri + 4*PI - startOri);
				}
				else{
					// cout << "			222222222222222" << endl;
					point.y += gps_vel*scan_period*( scan_angle - startOri + 2*PI + ((scan_angle<endOri)?(2*PI):0) )/(endOri + 4*PI - startOri);
				}
			}
		}  
	}

	//Sync TR**************************
	size_t cloudSizeTR = Cloud_TR.points.size();

	startOri = -std::atan2(Cloud_TR[0].y, Cloud_TR[0].x);
  	endOri = -std::atan2(Cloud_TR[cloudSizeTR - 1].y, Cloud_TR[cloudSizeTR - 1].x);

  	if(startOri<0) startOri += 2*PI;
  	if(endOri<0) endOri += 2*PI;

	for (size_t i = 0; i < cloudSizeTR; i++) {
		pcl::PointXYZI& point = Cloud_TR.points[i];
		double mm = point.y;
		double scan_angle = -atan2(point.y, point.x);
		if(scan_angle<0) scan_angle += 2*PI;
		if(startOri<endOri){
			if(endOri - startOri > PI){
				point.y += gps_vel*scan_period*(scan_angle - startOri)/(endOri - startOri);
				// cout << "endOri - startOri >= PI" << endl;
			}
			else{
				// cout << "endOri - startOri < PI" << endl;
				if(i < cloudSizeTR/2){	
					point.y += gps_vel*scan_period*(scan_angle - startOri + ((scan_angle>=startOri)?0:(2*PI)) )/(endOri - startOri + 2*PI);
					// cout << "			1111111111111111111111111:   " << point.y - mm << endl;//he
				}
				else {
					point.y += gps_vel*scan_period*(scan_angle - startOri + ((scan_angle < endOri)?2*PI:0) )/(endOri - startOri + 2*PI);
					// cout << "			222222222222222:   " << point.y - mm << endl;
				}
			}
		} 
		else {
			if (startOri - endOri < PI){
				point.y += gps_vel*scan_period*( ((scan_angle<=endOri)?(scan_angle+2*PI):scan_angle) - startOri)/(endOri+2*PI - startOri);
				// cout << "startOri - endOri < PI" << endl;
			}
			else{
				// cout << "startOri - endOri >= PI" << endl;
				if(i < cloudSizeTR/2){
					// cout << "			1111111111111111111111111" << endl;
					point.y += gps_vel*scan_period*( scan_angle - startOri + ((scan_angle>=startOri)?0:(2*PI)) )/(endOri + 4*PI - startOri);
				}
				else{
					// cout << "			222222222222222" << endl;
					point.y += gps_vel*scan_period*( scan_angle - startOri + 2*PI + ((scan_angle<endOri)?(2*PI):0) )/(endOri + 4*PI - startOri);
				}
			}
		}   
		point.y -= displaceTR;
	}
	
	sensor_msgs::PointCloud2 msy_cloud;
	pcl::toROSMsg(Cloud_TL, msy_cloud);
	msy_cloud.header.stamp = cloud_msg_2->header.stamp;
	msy_cloud.header.frame_id = "velodyne";
	pub_lidar1.publish(msy_cloud);
	// ROS_INFO("Transform Completed"); 

	pcl::toROSMsg(Cloud_TC, msy_cloud);
	msy_cloud.header.stamp = cloud_msg_2->header.stamp;
	msy_cloud.header.frame_id = "velodyne";
	pub_lidar2.publish(msy_cloud);
	// ROS_INFO("Transform Completed"); 

	pcl::toROSMsg(Cloud_TR, msy_cloud);
	msy_cloud.header.stamp = cloud_msg_2->header.stamp;
	msy_cloud.header.frame_id = "velodyne";
	pub_lidar3.publish(msy_cloud);
	// ROS_INFO("Transform Completed"); 

 //  	pcl::PointCloud<pcl::PointXYZI>::Ptr calib(new pcl::PointCloud<pcl::PointXYZI>());

	// size_t cloudSize1 = Cloud_TL.points.size();
	// size_t cloudSize2 = Cloud_TC.points.size();
	// size_t cloudSize3 = Cloud_TR.points.size();

	// for (size_t i = 0; i < cloudSize1; i++) {
	// calib->push_back(Cloud_TL[i]);
	// }

	// for (size_t i = 0; i < cloudSize2; i++) {
	// calib->push_back(Cloud_TC[i]);
	// }

	// for (size_t i = 0; i < cloudSize3; i++) {
	// calib->push_back(Cloud_TR[i]);
	// }

	// sensor_msgs::PointCloud2 msy_cloud;
	// pcl::toROSMsg(*calib, msy_cloud);
	// msy_cloud.header.stamp = cloud_msg_2->header.stamp;
	// msy_cloud.header.frame_id = "velodyne";
	// pub_lidar.publish(msy_cloud);
	// ROS_INFO("Transform Completed"); 
	// std::cout<< calib->points.size()<<std::endl;




	//TODO: Code for specific image_lidar process

};


int main(int argc,char ** argv){
ros::init(argc, argv, "sensor_sync");

ros::NodeHandle nh;

ros::Subscriber sub_vel = nh.subscribe("/gps/vel", 10, &gps);

pub_lidar1 = nh.advertise<sensor_msgs::PointCloud2>("/LIDAR1", 10);
pub_lidar2 = nh.advertise<sensor_msgs::PointCloud2>("/LIDAR2", 10);
pub_lidar3 = nh.advertise<sensor_msgs::PointCloud2>("/LIDAR3", 10);

message_filters::Subscriber<sensor_msgs::PointCloud2> sync_lidar1(nh, "Transformed_CloudTL", 10);
message_filters::Subscriber<sensor_msgs::PointCloud2> sync_lidar2(nh, "Transformed_CloudTC", 10);
message_filters::Subscriber<sensor_msgs::PointCloud2> sync_lidar3(nh, "Transformed_CloudTR", 10);
// message_filters::Subscriber<sensor_msgs::Image> sync_image(nh, "/ladybug_camera/camera4/image_raw", 10);

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sync_lidar1, sync_lidar2, sync_lidar3);
sync.registerCallback(boost::bind(&callback_lidar_sync, _1, _2, _3));
// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
// message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sync_lidar1, sync_lidar2, sync_lidar3, sync_image);
// sync.registerCallback(boost::bind(&callback_lidar_sync, _1, _2, _3, _4));


// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, geometry_msgs::TwistWithCovarianceStamped> MySyncPolicy;
// message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sync_lidar1, sync_lidar2, sync_lidar3, sync_gps);
// sync.registerCallback(boost::bind(&callback_lidar_sync, _1, _2, _3, _4));
ros::spin();
}


