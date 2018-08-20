#include "gicp_calibrate.h"


void callback_lidar_sync(const sensor_msgs::PointCloud2ConstPtr& cloud_msg_1, 
                        const sensor_msgs::PointCloud2ConstPtr& cloud_msg_2){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCloud_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCloud_2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*cloud_msg_1,*ptrCloud_1);
    pcl::fromROSMsg(*cloud_msg_2,*ptrCloud_2);

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(ptrCloud_2);
    icp.setInputTarget(ptrCloud_1);
    pcl::PointCloud<pcl::PointXYZ> Final; 
    std::cout << "start aligning "<< std::endl;
    icp.align(Final); 
    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
};

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "velodyne_sub");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_1(nh, "ns4/velodyne_points", 1000);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_2(nh, "ns1/velodyne_points", 1000);
 
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_1, sub_2);
  sync.registerCallback(boost::bind(&callback_lidar_sync, _1, _2));
  
  pub_velo = nh.advertise<sensor_msgs::PointCloud2>("/vlp_fuse", 1);
  pub_velo_wo_speed = nh.advertise<sensor_msgs::PointCloud2>("/vlp_fuse_wo_speed", 1);

  ros::spin();
  // Spin
}
