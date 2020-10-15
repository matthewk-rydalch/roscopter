#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <math.h>
#include <array>
#include <ros/ros.h>
#include <rosflight_msgs/Command.h>
#include <rosflight_msgs/Status.h>
#include <controller/simple_pid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <stdint.h>
// #include <dynamic_reconfigure/server.h>
// #include <roscopter/ControllerConfig.h>
#include "roscopter_utils/yaml.h"

namespace controller
{

typedef struct
{
  double pn;
  double pe;
  double pd;

  double phi;
  double theta;
  double psi;

  double u;
  double v;
  double w;

  double p;
  double q;
  double r;
} state_t;

typedef struct
{
  double pn;
  double pe;
  double pd;

  double phi;
  double theta;
  double psi;

  double x_dot;
  double y_dot;
  double z_dot;

  double r;

  double ax;
  double ay;
  double az;

  double throttle;
} command_t;

typedef struct
{
  double roll;
  double pitch;
  double yaw_rate;
  double throttle;
  double n_dot;
  double e_dot;
  double d_dot;
} max_t;

class Controller
{

public:

  Controller();
  void load(const std::string &filename);
  void setPIDXDot(double P, double I, double D, double tau);
  void setPIDYDot(double P, double I, double D, double tau);
  void setPIDZDot(double P, double I, double D, double tau);
  void setPIDN(double P, double I, double D, double tau);
  void setPIDE(double P, double I, double D, double tau);
  void setPIDD(double P, double I, double D, double tau);
  void setPIDPsi(double P, double I, double D, double tau);

  max_t max_ = {};

  // Paramters
  double throttle_eq_;
  double mass_;
  double max_thrust_;
  double max_accel_xy_;
  double max_accel_z_;
  double min_altitude_;
  float throttle_down_ = 0.95;
  bool is_flying_;
  bool armed_;
  bool received_cmd_;
  bool use_feed_forward_ = false;
  bool is_landing_ = false;
  bool landed_ = false;

// protected:

  uint8_t control_mode_;

// private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

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

  // PID Controllers
  controller::SimplePID PID_x_dot_;
  controller::SimplePID PID_y_dot_;
  controller::SimplePID PID_z_dot_;
  controller::SimplePID PID_n_;
  controller::SimplePID PID_e_;
  controller::SimplePID PID_d_;
  controller::SimplePID PID_psi_;

  // Memory for sharing information between functions
  state_t xhat_ = {}; // estimate
  state_t base_hat_ = {}; //base(frame) state
  rosflight_msgs::Command command_;
  command_t xc_ = {}; // command
  double prev_time_;
//
  // Functions
  void stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void isFlyingCallback(const std_msgs::BoolConstPtr &msg);
  void cmdCallback(const rosflight_msgs::CommandConstPtr &msg);
  void statusCallback(const rosflight_msgs::StatusConstPtr &msg);
  void baseOdomCallback(const nav_msgs::OdometryConstPtr &msg);
  void useFeedForwardCallback(const std_msgs::BoolConstPtr &msg);
  void isLandingCallback(const std_msgs::BoolConstPtr &msg);
  void landedCallback(const std_msgs::BoolConstPtr &msg);

  void computeControl(double dt);
  void resetIntegrators();
  void publishCommand();
  double saturate(double x, double max, double min);
  void addFeedForwardTerm();

  Eigen::Matrix3d Rroll(double phi);
  Eigen::Matrix3d Rpitch(double theta);
  Eigen::Matrix3d Ryaw(double psi);
};
} //namespace controller

#endif
