
#include "transformation.h"

int main(int argc,char ** argv){
ros::init(argc, argv, "transform");

ros::NodeHandle nh;
 // ros::NodeHandle privateNode("~");

MSY_Denso::msy_multi_lidar transform_node;
transform_node.setup(nh);

transform_node.spin();
}


