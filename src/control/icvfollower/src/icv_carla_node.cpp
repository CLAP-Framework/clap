#include "icv_carla_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "icvCarlaControlnode");
  icv::icvCarlaControlNode ppn;
  ppn.Noderun();

  return 0;
}
