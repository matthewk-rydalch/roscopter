#ifndef VEL_CNTRL_ROS_H
#define VEL_CNTRL_ROS_H

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
#include <controller/vel_cntrl.h>

class Vel_Cntrl_Ros
{

public:

  Vel_Cntrl_Ros();

protected:
  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  controller::Vel_Cntrl control;

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
  ros::Subscriber target_estimate_sub_;
  ros::Subscriber is_landing_sub_;
  ros::Subscriber use_feed_forward_sub_;
  
  ros::Publisher command_pub_;

  //functions
  void init_controller();
  void stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void isFlyingCallback(const std_msgs::BoolConstPtr &msg);
  void cmdCallback(const rosflight_msgs::CommandConstPtr &msg);
  void statusCallback(const rosflight_msgs::StatusConstPtr &msg);
  void targetEstimateCallback(const nav_msgs::OdometryConstPtr &msg);
  void useFeedForwardCallback(const std_msgs::BoolConstPtr &msg);
  void isLandingCallback(const std_msgs::BoolConstPtr &msg);

  void publishCommand();

  double prev_time_;

  rosflight_msgs::Command command_;

  bool debug_Vel_Cntrl_Ros_{false};
  bool debug_init_controller_{false};
  bool debug_stateCallback_{false};
  bool debug_isFlyingCallback_{false};
  bool debug_cmdCallback_{false};
  bool debug_statusCallback_{false};
  bool debug_reconfigure_callback_{false};
  bool debug_publishCommand_{false};

  bool is_flying_{false};
  bool received_cmd_{false};
  bool armed_{false}; //TODO this may need to be left undefined
};

#endif
