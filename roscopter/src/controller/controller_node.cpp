#include <ros/ros.h>
#include "controller/controller_ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle nh;
  Controller_Ros Thing;

  ros::spin();

  return 0;
}
