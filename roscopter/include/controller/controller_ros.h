#ifndef CONTROLLER_ROS_H
#define CONTROLLER_ROS_H

#include <ros/ros.h>
#include <ros/package.h>
// #include <tf/tf.h>
// #include <stdint.h>
#include <dynamic_reconfigure/server.h>
#include <roscopter/ControllerConfig.h>
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

    // Dynamic Reconfigure Hooks
  dynamic_reconfigure::Server<roscopter::ControllerConfig> _server;
  dynamic_reconfigure::Server<roscopter::ControllerConfig>::CallbackType _func;
  void reconfigure_callback(roscopter::ControllerConfig& config,
                            uint32_t level);
};

#endif
