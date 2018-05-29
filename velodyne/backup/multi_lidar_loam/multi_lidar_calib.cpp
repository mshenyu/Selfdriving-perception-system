
#include "multi_lidar_calib.h"

int main(int argc,char ** argv){
ros::init(argc, argv, "one_lidar_read");

ros::NodeHandle nh;
 // ros::NodeHandle privateNode("~");

MSY_Denso::msy_multi_lidar case1;
case1.setup(nh);

case1.spin();
}


