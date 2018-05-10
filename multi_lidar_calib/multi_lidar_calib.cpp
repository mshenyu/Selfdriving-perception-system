
#include "multi_lidar_calib.h"
#include <Eigen/Core>
#include <Eigen/Dense>

// void msy_multi_lidar::lidar_read1( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg);
// void msy_multi_lidar::lidar_read3( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg);
// void msy_multi_lidar::lidar_read4( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg);
// void msy_multi_lidar::lidar_read5( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg);


int main(int argc,char ** argv){
ros::init(argc, argv, "one_lidar_read");

ros::NodeHandle nh;

msy_multi_lidar case1;
case1.setup(nh);

case1.spin();
}


void msy_multi_lidar::lidar_read1( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg){
	ROS_INFO_STREAM( " Hello Velodyne 1 " );

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud(new pcl::PointCloud<pcl::PointXYZI>);
  	pcl::fromROSMsg(*laserCloudMsg, *ptrCloud);
	  CloudFR_pt = ptrCloud;

    pcl::fromROSMsg(*laserCloudMsg, CloudFR);
  	// sensor_msgs::PointCloud2 msy_laserCloud;
  	// pcl::toROSMsg(*CloudFR_pt, msy_laserCloud);

  	// pub1.publish(msy_laserCloud);

}
void msy_multi_lidar::lidar_read3( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg){
	ROS_INFO_STREAM( " Hello Velodyne 3 " );
  	
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud(new pcl::PointCloud<pcl::PointXYZI>);
  	pcl::fromROSMsg(*laserCloudMsg, *ptrCloud);
	  CloudTF_pt = ptrCloud;

    pcl::fromROSMsg(*laserCloudMsg, CloudTF);
  	// sensor_msgs::PointCloud2 msy_laserCloud;
  	// pcl::toROSMsg(*CloudTF_pt, msy_laserCloud);

  	// pub3.publish(msy_laserCloud);

}
void msy_multi_lidar::lidar_read4( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg){
	ROS_INFO_STREAM( " Hello Velodyne 4 " );
  	
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud(new pcl::PointCloud<pcl::PointXYZI>);
  	pcl::fromROSMsg(*laserCloudMsg, *ptrCloud);
	  CloudTC_pt = ptrCloud;

    pcl::fromROSMsg(*laserCloudMsg, CloudTC);
  	// sensor_msgs::PointCloud2 msy_laserCloud;
  	// pcl::toROSMsg(*CloudTC_pt, msy_laserCloud);

  	// pub4.publish(msy_laserCloud);

}
void msy_multi_lidar::lidar_read5( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg){
	ROS_INFO_STREAM( " Hello Velodyne 5 " );
  	
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud(new pcl::PointCloud<pcl::PointXYZI>);
  	pcl::fromROSMsg(*laserCloudMsg, *ptrCloud);
	  CloudTR_pt = ptrCloud;

    pcl::fromROSMsg(*laserCloudMsg, CloudTR);
  	// sensor_msgs::PointCloud2 msy_laserCloud;
  	// pcl::toROSMsg(*CloudTR_pt, msy_laserCloud);

  	// pub5.publish(msy_laserCloud);

}


bool msy_multi_lidar::process()
{
  ROS_INFO_STREAM( "      Processing " );

  Eigen::Matrix4f transformFR2TC;
  Eigen::Matrix4f transformTF2TC;
  Eigen::Matrix4f transformTR2TC;

  transformTF2TC <<   0.955288, -0.00374613, -0.29281, -0.47146,
                      0.00565324, 0.999969, 0.00542592, 0.00720199,
                      0.292796, -0.00690181, 0.955046, -0.0680386,
                      0,      0,      0,      1;
  transformTR2TC <<   0.948306, -0.016546, 0.316926, 0.462375,
                      0.0225407, 0.999629, -0.0152577, 0.007567,
                     -0.316556, 0.0216127, 0.948328, -0.0674528,
                      0,      0,      0,      1;

  transformFR2TC <<   0.822451,  -0.538836,   0.182294,     1.4598,
                      0.552549,   0.832905, -0.0309693,  -0.395969,
                     -0.135146,   0.126197,   0.982756,  -0.290385,
                             0,          0,          0,          1;

  pcl::transformPointCloud(CloudFR, CloudFR, transformFR2TC);
  pcl::transformPointCloud(CloudTF, CloudTF, transformTF2TC);
  pcl::transformPointCloud(CloudTR, CloudTR, transformTR2TC);
  
  sensor_msgs::PointCloud2 msy_laserCloud;
  pcl::toROSMsg(CloudFR, msy_laserCloud);
  pub1.publish(msy_laserCloud);

  pcl::toROSMsg(CloudTF, msy_laserCloud);
  pub3.publish(msy_laserCloud);

  pcl::toROSMsg(CloudTC, msy_laserCloud);
  pub4.publish(msy_laserCloud);

  pcl::toROSMsg(CloudTR, msy_laserCloud);
  pub5.publish(msy_laserCloud);

  return true;
}