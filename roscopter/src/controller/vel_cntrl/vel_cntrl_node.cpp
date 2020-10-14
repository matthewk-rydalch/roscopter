#include <ros/ros.h>
#include "controller/vel_cntrl.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cntrl_node");
  ros::NodeHandle nh;

  Vel_Cntrl Thing;

  ros::spin();

  return 0;
}