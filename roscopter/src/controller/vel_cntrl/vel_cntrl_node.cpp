#include <ros/ros.h>
#include "controller/vel_cntrl_ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cntrl_node");
  ros::NodeHandle nh;

  Vel_Cntrl_Ros Thing;

  ros::spin();

  return 0;
}