#include "pointpillars.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_pointpillars_detect");
  Pointpillars node;
  node.run();
  
  // node.test_run();
  ros::spin();

  return 0;
}
