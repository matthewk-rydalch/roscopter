#ifndef CONTROLLER_ROS_H
#define CONTROLLER_ROS_H

#include <ros/ros.h>
#include <ros/package.h>
// #include <tf/tf.h>
// #include <stdint.h>
#include <iostream>
#include <controller/controller.h>

class Controller_Ros
{

public:

  Controller_Ros();

private:
  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  controller::Controller control;
};

#endif
