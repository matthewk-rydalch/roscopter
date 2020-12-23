#include <ros/ros.h>
#include "ekf/target_ekf_ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "estimator");

  roscopter::ekf::EKF_ROS estimator;

  ros::spin();

  return 0;
}

