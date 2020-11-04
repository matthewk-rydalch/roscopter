#ifndef FF_CNTRL_ROS_H
#define FF_CNTRL_ROS_H

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
#include <controller/ff_cntrl.h>

class Ff_Cntrl_Ros
{
  //TODO: Make this inherit from controller_ros.  Need to make the stateCallback virtual
  // and overide it with a similar function that calls computeControlFeedForward rather than computecontrol.

public:

  Ff_Cntrl_Ros();

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  controller::Ff_Cntrl control;

  dynamic_reconfigure::Server<roscopter::ControllerConfig> _server;
  dynamic_reconfigure::Server<roscopter::ControllerConfig>::CallbackType _func;
  void reconfigure_callback(roscopter::ControllerConfig& config,
                            uint32_t level);

  ros::Subscriber state_sub_;
  ros::Subscriber is_flying_sub_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber status_sub_;
  ros::Subscriber target_estimate_sub_;
  ros::Subscriber is_landing_sub_;
  ros::Subscriber add_integrator_sub_;
  ros::Subscriber use_feed_forward_sub_;
  ros::Subscriber landed_sub_;
  
  ros::Publisher command_pub_;

  void init_controller();
  void stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void isFlyingCallback(const std_msgs::BoolConstPtr &msg);
  void cmdCallback(const rosflight_msgs::CommandConstPtr &msg);
  void statusCallback(const rosflight_msgs::StatusConstPtr &msg);
  void targetEstimateCallback(const nav_msgs::OdometryConstPtr &msg);
  void useFeedForwardCallback(const std_msgs::BoolConstPtr &msg);
  void isLandingCallback(const std_msgs::BoolConstPtr &msg);
  void addIntegratorCallback(const std_msgs::BoolConstPtr &msg);
  void landedCallback(const std_msgs::BoolConstPtr &msg);

  void publishCommand();

  double prev_time_;

  rosflight_msgs::Command command_;

  bool debug_Ff_Cntrl_Ros_{false};
  bool debug_init_controller_{false};
  bool debug_stateCallback_{false};
  bool debug_isFlyingCallback_{false};
  bool debug_cmdCallback_{false};
  bool debug_statusCallback_{false};
  bool debug_reconfigure_callback_{false};
  bool debug_publishCommand_{false};

  bool is_flying_{false};
  bool received_cmd_{false};
  bool armed_{false}; //TODO this may need to be left undefined?
};

#endif
