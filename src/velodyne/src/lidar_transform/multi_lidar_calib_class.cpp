#include "multi_lidar_calib.h"

namespace MSY_Denso{

msy_multi_lidar::msy_multi_lidar()
{
    ;
}

void msy_multi_lidar::lidar_read3( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg){
    CloudTL.clear();
    TL_hascome = true;
    _timeTL = laserCloudMsg->header.stamp;
    pcl::fromROSMsg(*laserCloudMsg, CloudTL);
    transform_rx = 0.0146647;
    transform_ry = 0.337381;
    transform_rz = 0.0304001;
    transform_tx = 0.493848;
    transform_ty = 0.0000530883;
    transform_tz = 0.0782908;
    FinalTransform(CloudTL);
    sensor_msgs::PointCloud2 msy_laserCloud;
    pcl::toROSMsg(CloudTL, msy_laserCloud);
    msy_laserCloud.header.stamp = _timeTL;
    msy_laserCloud.header.frame_id = "velodyne";
    pub3.publish(msy_laserCloud);
}
void msy_multi_lidar::lidar_read4( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg){
    CloudTC.clear();
    TC_hascome = true;
    _timeTC = laserCloudMsg->header.stamp;
    pcl::fromROSMsg(*laserCloudMsg, CloudTC);
    sensor_msgs::PointCloud2 msy_laserCloud;
    pcl::toROSMsg(CloudTC, msy_laserCloud);
    msy_laserCloud.header.stamp = _timeTC;
    msy_laserCloud.header.frame_id = "velodyne";
    pub4.publish(msy_laserCloud);
}
void msy_multi_lidar::lidar_read5( const sensor_msgs::PointCloud2ConstPtr & laserCloudMsg){
    CloudTR.clear();
    TR_hascome = true;
    _timeTR = laserCloudMsg->header.stamp;
    pcl::fromROSMsg(*laserCloudMsg, CloudTR);
    transform_rx = -0.0264753;
    transform_ry = -0.283988;
    transform_rz = 0.00813624;
    transform_tx = -0.482452;
    transform_ty = 0.0127083;
    transform_tz = 0.0805756;
    FinalTransform(CloudTR);
    sensor_msgs::PointCloud2 msy_laserCloud;
    pcl::toROSMsg(CloudTR, msy_laserCloud);
    msy_laserCloud.header.stamp = _timeTR;
    msy_laserCloud.header.frame_id = "velodyne";
    pub5.publish(msy_laserCloud);
}

void msy_multi_lidar::setup(ros::NodeHandle& nh){
  TL_hascome = false;
  TC_hascome = false;
  TR_hascome = false;
  sub3 = nh.subscribe("ns3/velodyne_points", 10, &msy_multi_lidar::lidar_read3, this);
  pub3 = nh.advertise<sensor_msgs::PointCloud2>("/Transformed_CloudTL", 10);
  sub4 = nh.subscribe("ns4/velodyne_points", 10, &msy_multi_lidar::lidar_read4, this);
  pub4 = nh.advertise<sensor_msgs::PointCloud2>("/Transformed_CloudTC", 10);
  sub5 = nh.subscribe("ns5/velodyne_points", 10, &msy_multi_lidar::lidar_read5, this);
  pub5 = nh.advertise<sensor_msgs::PointCloud2>("/Transformed_CloudTR", 10);
  pub_calib = nh.advertise<sensor_msgs::PointCloud2>("/calib0", 10);
}

void msy_multi_lidar::spin(){
    ros::Rate rate(100);
    // ros::spinOnce();
    rate.sleep();
    // loop until shutdown
    while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
    }
}


void msy_multi_lidar::FinalTransform(pcl::PointCloud<pcl::PointXYZI>& cloud)
{
  size_t cloudSize = cloud.points.size();
  for (size_t i = 0; i < cloudSize; i++) {
    pcl::PointXYZI& point = cloud.points[i];
    Angle rx = - transform_rx;
    Angle ry = - transform_ry;
    Angle rz = - transform_rz;
    rotateZXY(point, rz, rx, ry);
    point.x = point.x - transform_tx;
    point.y = point.y - transform_ty;
    point.z = point.z - transform_tz;
  }
}

} //namespace end