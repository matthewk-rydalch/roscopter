#include <controller/controller_ros.h>
#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <fstream>

namespace controller
{
  Controller::Controller() :
    nh_(ros::NodeHandle()),
    nh_private_("~")
  {
    // Retrieve global MAV equilibrium throttle. This is the only MAV specific
    // parameter that is required
    ros::NodeHandle nh_mav(ros::this_node::getNamespace());
    if (!nh_private_.getParam("equilibrium_throttle", throttle_eq_))
      ROS_ERROR("[Controller] MAV equilibrium_throttle not found!");

    // Calculate max accelerations. Assuming that equilibrium throttle produces
    // 1 g of acceleration and a linear thrust model, these max acceleration
    // values are computed in g's as well.
    max_accel_z_ = 1.0 / throttle_eq_;
    max_accel_xy_ = sin(acos(throttle_eq_)) / throttle_eq_ / sqrt(2.);

    is_flying_ = false;
    received_cmd_ = false;

    nh_private_.getParam("max_roll", max_.roll);
    nh_private_.getParam("max_pitch", max_.pitch);
    nh_private_.getParam("max_yaw_rate", max_.yaw_rate);
    nh_private_.getParam("max_throttle", max_.throttle);
    nh_private_.getParam("max_n_dot", max_.n_dot);
    nh_private_.getParam("max_e_dot", max_.e_dot);
    nh_private_.getParam("max_d_dot", max_.d_dot);

    nh_private_.getParam("min_altitude", min_altitude_);

    _func = boost::bind(&Controller::reconfigure_callback, this, _1, _2);
    _server.setCallback(_func);

    // Set up Publishers and Subscriber
    command_pub_ = nh_.advertise<rosflight_msgs::Command>("command", 1);

    state_sub_ = nh_.subscribe("estimate", 1, &Controller::stateCallback, this);
    is_flying_sub_ = nh_.subscribe("is_flying", 1, &Controller::isFlyingCallback, this);
    cmd_sub_ = nh_.subscribe("high_level_command", 1, &Controller::cmdCallback, this);
    status_sub_ = nh_.subscribe("status", 1, &Controller::statusCallback, this);
    base_odom_sub_ = nh_.subscribe("base_odom", 1, &Controller::baseOdomCallback, this);
    use_feed_forward_sub_ = nh_.subscribe("use_base_feed_forward_vel", 1, &Controller::useFeedForwardCallback, this);
    is_landing_sub_ = nh_.subscribe("is_landing", 1, &Controller::isLandingCallback, this);
    landed_sub_ = nh_.subscribe("landed", 1, &Controller::landedCallback, this);

  }

  void Controller::stateCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    
    static double prev_time = 0;
    if(prev_time == 0)
    {
      prev_time = msg->header.stamp.toSec();
      return;
    }

    // Calculate time
    double now = msg->header.stamp.toSec();
    double dt = now - prev_time;
    prev_time = now;

    if(dt <= 0)
      return;

    // This should already be coming in NED
    xhat_.pn = msg->pose.pose.position.x;
    xhat_.pe = msg->pose.pose.position.y;
    xhat_.pd = msg->pose.pose.position.z;

    xhat_.u = msg->twist.twist.linear.x;
    xhat_.v = msg->twist.twist.linear.y;
    xhat_.w = msg->twist.twist.linear.z;

    // Convert Quaternion to RPY
    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, tf_quat);
    tf::Matrix3x3(tf_quat).getRPY(xhat_.phi, xhat_.theta, xhat_.psi);
    xhat_.theta = xhat_.theta;
    xhat_.psi = xhat_.psi;

    xhat_.p = msg->twist.twist.angular.x;
    xhat_.q = msg->twist.twist.angular.y;
    xhat_.r = msg->twist.twist.angular.z;
    
    if(is_flying_ && armed_ && received_cmd_)
    {
      ROS_WARN_ONCE("CONTROLLER ACTIVE");
      computeControl(dt);
      publishCommand();
    }
    else
    {
      resetIntegrators();
      prev_time_ = msg->header.stamp.toSec();
    }
  }

  void Controller::isFlyingCallback(const std_msgs::BoolConstPtr &msg)
  {
    is_flying_ = msg->data;
  }

  void Controller::statusCallback(const rosflight_msgs::StatusConstPtr &msg)
  {
    armed_ = msg->armed;
  }

  void Controller::cmdCallback(const rosflight_msgs::CommandConstPtr &msg)
  {
    switch(msg->mode)
    {
      case rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE:
        xc_.pn = msg->x;
        xc_.pe = msg->y;
        xc_.pd = -msg->F;
        xc_.psi = msg->z;
        control_mode_ = msg->mode;
        break;
      case rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE:
        xc_.x_dot = msg->x;
        xc_.y_dot = msg->y;
        xc_.pd = -msg->F;
        xc_.r = msg->z;
        control_mode_ = msg->mode;
        break;
      case rosflight_msgs::Command::MODE_XACC_YACC_YAWRATE_AZ:
        xc_.ax = msg->x;
        xc_.ay = msg->y;
        xc_.az = msg->F;
        xc_.r = msg->z;
        control_mode_ = msg->mode;
        break;
      default:
        ROS_ERROR("roscopter/controller: Unhandled command message of type %d",
                  msg->mode);
        break;
    }

    if (!received_cmd_)
      received_cmd_ = true;
  }

  void Controller::baseOdomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    // // Convert Quaternion to RPY
    // tf::Quaternion tf_quat;
    // tf::quaternionMsgToTF(msg->pose.pose.orientation, tf_quat);
    // tf::Matrix3x3(tf_quat).getRPY(base_hat_.phi, base_hat_.theta, base_hat_.psi);
    // base_hat_.psi = -base_hat_.psi; //have to negate to go from Counter Clockwise positive rotation to Clockwise;

    // double sinp = sin(base_hat_.psi);
    // double cosp = cos(base_hat_.psi);

    double u = msg->twist.twist.linear.x;
    double v = msg->twist.twist.linear.y;
    double w = msg->twist.twist.linear.z;

    base_hat_.u = u;
    base_hat_.v = v;
    base_hat_.w = w;

    // base_hat_.r = -msg->twist.angular.z; //have to negate to go from Counter Clockwise positive rotation to Clockwise
  }

  void Controller::useFeedForwardCallback(const std_msgs::BoolConstPtr &msg)
  {
    use_feed_forward_ = msg->data;
  }

  void Controller::isLandingCallback(const std_msgs::BoolConstPtr &msg)
  {
    is_landing_ = msg->data;
  }

  void Controller::landedCallback(const std_msgs::BoolConstPtr &msg)
  {
    landed_ = msg->data;
  }

  void Controller::reconfigure_callback(roscopter::ControllerConfig& config,
                                        uint32_t level)
  {
    double P, I, D, tau;
    tau = config.tau;
    P = config.x_dot_P;
    I = config.x_dot_I;
    D = config.x_dot_D;
    PID_x_dot_.setGains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);

    P = config.y_dot_P;
    I = config.y_dot_I;
    D = config.y_dot_D;
    PID_y_dot_.setGains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);

    P = config.z_dot_P;
    I = config.z_dot_I;
    D = config.z_dot_D;
    // set max z accelerations so that we can't fall faster than 1 gravity
    PID_z_dot_.setGains(P, I, D, tau, 1.0, -max_accel_z_);

    P = config.north_P;
    I = config.north_I;
    D = config.north_D;
    max_.n_dot = config.max_n_dot;
    PID_n_.setGains(P, I, D, tau, max_.n_dot, -max_.n_dot);

    P = config.east_P;
    I = config.east_I;
    D = config.east_D;
    max_.e_dot = config.max_e_dot;
    PID_e_.setGains(P, I, D, tau, max_.e_dot, -max_.e_dot);

    P = config.down_P;
    I = config.down_I;
    D = config.down_D;
    max_.d_dot = config.max_d_dot;
    PID_d_.setGains(P, I, D, tau, max_.d_dot, -max_.d_dot);

    P = config.psi_P;
    I = config.psi_I;
    D = config.psi_D;
    PID_psi_.setGains(P, I, D, tau);

    max_.roll = config.max_roll;
    max_.pitch = config.max_pitch;
    max_.yaw_rate = config.max_yaw_rate;
    max_.throttle = config.max_throttle;

    max_.n_dot = config.max_n_dot;
    max_.e_dot = config.max_e_dot;
    max_.d_dot = config.max_d_dot;

    throttle_eq_ = config.equilibrium_throttle;

    ROS_INFO("new gains");

    resetIntegrators();
  }

  void Controller::publishCommand()
  {
    command_.header.stamp = ros::Time::now();
    command_pub_.publish(command_);
  }

  void Controller::resetIntegrators()
  {
    PID_x_dot_.clearIntegrator();
    PID_y_dot_.clearIntegrator();
    PID_z_dot_.clearIntegrator();
    PID_n_.clearIntegrator();
    PID_e_.clearIntegrator();
    PID_d_.clearIntegrator();
    PID_psi_.clearIntegrator();
  }
}
