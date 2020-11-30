#include <ros/ros.h>
#include "controller/ff_cntrl_ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cntrl_node");
  ros::NodeHandle nh;

  Ff_Cntrl_Ros Thing;

  ros::spin();

  return 0;
}