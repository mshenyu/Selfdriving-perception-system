#include "transformation.h"

namespace MSY_Denso{



void setup(ros::NodeHandle){




sub1 = nh.subscribe("ns1/velodyne_points", 10, &msy_multi_lidar::lidar_read1, this);
pub1 = nh.advertise<sensor_msgs::PointCloud2>("/transferred_FR", 10);


	
}






















}


