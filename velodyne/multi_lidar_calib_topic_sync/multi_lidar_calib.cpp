
#include "multi_lidar_calib.h"
#include <Eigen/Core>
#include <Eigen/Dense>


using namespace sensor_msgs;
using namespace message_filters;

void callback(const sensor_msgs:: PointCloud2ConstPtr& point_cloud_FR,
              const sensor_msgs:: PointCloud2ConstPtr& point_cloud_FL,
              const sensor_msgs:: PointCloud2ConstPtr& point_cloud_TL,
              const sensor_msgs:: PointCloud2ConstPtr& point_cloud_TC,
              const sensor_msgs:: PointCloud2ConstPtr& point_cloud_TR)
{
    ROS_INFO("Calibrating");

    pcl::fromROSMsg(*point_cloud_FR, CloudFR);
    pcl::fromROSMsg(*point_cloud_FL, CloudFL);
    pcl::fromROSMsg(*point_cloud_TL, CloudTL);
    pcl::fromROSMsg(*point_cloud_TC, CloudTC);
    pcl::fromROSMsg(*point_cloud_TR, CloudTR);

      Eigen::Matrix4f transformFR2TC;
      Eigen::Matrix4f transformTL2TC;
      Eigen::Matrix4f transformTR2TC;

      // transformFR2TC <<   0.822451,  -0.538836,   0.182294,     1.4598,
      //                     0.552549,   0.832905, -0.0309693,  -0.395969,
      //                    -0.135146,   0.126197,   0.982756,  -0.290385,
      //                            0,          0,          0,          1;

      transformTL2TC <<   0.955288, -0.00374613, -0.29281, -0.47146,
                          0.00565324, 0.999969, 0.00542592, 0.00720199,
                          0.292796, -0.00690181, 0.955046, -0.0680386,
                          0,      0,      0,      1;
      transformTR2TC <<   0.948306, -0.016546, 0.316926, 0.462375,
                          0.0225407, 0.999629, -0.0152577, 0.007567,
                         -0.316556, 0.0216127, 0.948328, -0.0674528,
                          0,      0,      0,      1;


      // pcl::transformPointCloud(CloudFR, CloudFR, transformFR2TC);
      pcl::transformPointCloud(CloudTL, CloudTL, transformTL2TC);
      pcl::transformPointCloud(CloudTR, CloudTR, transformTR2TC);

    sensor_msgs::PointCloud2 msy_transferred;
    pcl::toROSMsg(CloudFR, msy_transferred);
    pub1.publish(msy_transferred);
    pcl::toROSMsg(CloudFL, msy_transferred);
    pub2.publish(msy_transferred);
    pcl::toROSMsg(CloudTL, msy_transferred);
    pub3.publish(msy_transferred);
    pcl::toROSMsg(CloudTC, msy_transferred);
    pub4.publish(msy_transferred);
    pcl::toROSMsg(CloudTR, msy_transferred);
    pub5.publish(msy_transferred);
}

int main(int argc,char ** argv){
ros::init(argc, argv, "multi_lidar_calib");

ros::NodeHandle nh;

pub1 = nh.advertise<sensor_msgs::PointCloud2>("/transferred_FR", 1);
pub2 = nh.advertise<sensor_msgs::PointCloud2>("/transferred_FL", 1);
pub3 = nh.advertise<sensor_msgs::PointCloud2>("/transferred_TL", 1);
pub4 = nh.advertise<sensor_msgs::PointCloud2>("/transferred_TC", 1);
pub5 = nh.advertise<sensor_msgs::PointCloud2>("/transferred_TR", 1);

message_filters::Subscriber<sensor_msgs::PointCloud2> Velodyne_sub1(nh, "ns1/velodyne_points", 1);
message_filters::Subscriber<sensor_msgs::PointCloud2> Velodyne_sub2(nh, "ns2/velodyne_points", 1);
message_filters::Subscriber<sensor_msgs::PointCloud2> Velodyne_sub3(nh, "ns3/velodyne_points", 1);
message_filters::Subscriber<sensor_msgs::PointCloud2> Velodyne_sub4(nh, "ns4/velodyne_points", 1);
message_filters::Subscriber<sensor_msgs::PointCloud2> Velodyne_sub5(nh, "ns5/velodyne_points", 1);

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                        sensor_msgs::PointCloud2,
                                                        sensor_msgs::PointCloud2,
                                                        sensor_msgs::PointCloud2,
                                                        sensor_msgs::PointCloud2> MySyncPolicy;

message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20),
                                                   Velodyne_sub1,
                                                   Velodyne_sub2,
                                                   Velodyne_sub3,
                                                   Velodyne_sub4,
                                                   Velodyne_sub5);
sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));
ros::spin();

return 0;
}