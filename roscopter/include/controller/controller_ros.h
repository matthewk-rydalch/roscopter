#ifndef CONTROLLER_ROS_H
#define CONTROLLER_ROS_H

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <stdint.h>
#include <rosflight_msgs/Command.h>
#include <rosflight_msgs/Status.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
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

  // Publishers and Subscribers
  ros::Subscriber state_sub_;
  ros::Subscriber is_flying_sub_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber status_sub_;
  ros::Subscriber base_odom_sub_;
  ros::Subscriber is_landing_sub_;
  ros::Subscriber use_feed_forward_sub_;
  ros::Subscriber landed_sub_;

  ros::Publisher command_pub_;

  //functions
  void init_controller();
  void stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void isFlyingCallback(const std_msgs::BoolConstPtr &msg);
  void cmdCallback(const rosflight_msgs::CommandConstPtr &msg);
  void statusCallback(const rosflight_msgs::StatusConstPtr &msg);
  void publishCommand();

  double prev_time_;

  rosflight_msgs::Command command_;
};

#endif
