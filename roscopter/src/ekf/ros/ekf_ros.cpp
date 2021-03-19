// BSD 3-Clause License
//
// Copyright (c) 2017, James Jackson, BYU MAGICC Lab, Provo UT
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <ros/package.h>

#include "ekf/state.h"
#include "ekf/ekf_ros.h"
#include "roscopter_utils/yaml.h"
#include "roscopter_utils/gnss.h"

using namespace Eigen;

namespace roscopter::ekf
{
  EKF_ROS::EKF_ROS() :
    nh_(), nh_private_("~")
  {}

  EKF_ROS::~EKF_ROS()
  {}

  void EKF_ROS::initROS()
  {

    std::string roscopter_path = ros::package::getPath("roscopter");
    std::string parameter_filename = nh_private_.param<std::string>("param_filename", roscopter_path + "/params/ekf.yaml");

    init(parameter_filename);

    odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
    euler_rad_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("euler_radians", 1);
    euler_deg_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("euler_degrees", 1);
    imu_bias_pub_ = nh_.advertise<sensor_msgs::Imu>("imu_bias", 1);

    imu_sub_ = nh_.subscribe("imu", 100, &EKF_ROS::imuCallback, this);

  #ifdef UBLOX
    ublox_gnss_sub_ = nh_.subscribe("ublox_gnss", 10, &EKF_ROS::gnssCallbackUblox, this);
    ublox_relpos_sub_ = nh_.subscribe("ublox_relpos", 10, &EKF_ROS::gnssCallbackRelPos, this);
    base_relPos_pub_ = nh_.advertise<geometry_msgs::PointStamped>("base_relPos", 1);
    std::cout << "UBLOX is defined \n";
  #endif
  #ifdef INERTIAL_SENSE
    is_gnss_sub_ = nh_.subscribe("is_gnss", 10, &EKF_ROS::gnssCallbackInertialSense, this);
  #endif

    ros_initialized_ = true;
  }

  void EKF_ROS::init(const std::string &param_file)
  {
    ekf_.load(param_file);

    // Load Sensor Noise Parameters
    double acc_stdev, gyro_stdev;
    get_yaml_node("accel_noise_stdev", param_file, acc_stdev);
    get_yaml_node("gyro_noise_stdev", param_file, gyro_stdev);

    imu_R_.setZero();
    imu_R_.topLeftCorner<3,3>() = acc_stdev * acc_stdev * I_3x3;
    imu_R_.bottomRightCorner<3,3>() = gyro_stdev * gyro_stdev * I_3x3;

    double pos_stdev, att_stdev;
    get_yaml_node("position_noise_stdev", param_file, pos_stdev);
    get_yaml_node("attitude_noise_stdev", param_file, att_stdev);
    mocap_R_ << pos_stdev * pos_stdev * I_3x3,   Matrix3d::Zero(),
        Matrix3d::Zero(),   att_stdev * att_stdev * I_3x3;

    get_yaml_node("manual_gps_noise", param_file, manual_gps_noise_);
    if (manual_gps_noise_)
    {
      get_yaml_node("gps_horizontal_stdev", param_file, gps_horizontal_stdev_);
      get_yaml_node("gps_vertical_stdev", param_file, gps_vertical_stdev_);
      get_yaml_node("gps_speed_stdev", param_file, gps_speed_stdev_);
    }

    get_yaml_node("manual_compassing_noise", param_file, manual_compassing_noise_);
    if(manual_compassing_noise_)
    {
      get_yaml_node("rtk_compassing_noise_stdev", param_file, rtk_compassing_noise_stdev_);
      compassing_R_ = rtk_compassing_noise_stdev_ * rtk_compassing_noise_stdev_;
    }

    start_time_.fromSec(0.0);
  }

  void EKF_ROS::publishEstimates(const sensor_msgs::ImuConstPtr &msg)
  {

    // Pub Odom
    odom_msg_.header = msg->header;

    const State state_est = ekf_.x();
    odom_msg_.pose.pose.position.x = state_est.p(0);
    odom_msg_.pose.pose.position.y = state_est.p(1);
    odom_msg_.pose.pose.position.z = state_est.p(2);

    odom_msg_.pose.pose.orientation.w = state_est.q.w();
    odom_msg_.pose.pose.orientation.x = state_est.q.x();
    odom_msg_.pose.pose.orientation.y = state_est.q.y();
    odom_msg_.pose.pose.orientation.z = state_est.q.z();

    odom_msg_.twist.twist.linear.x = state_est.v(0);
    odom_msg_.twist.twist.linear.y = state_est.v(1);
    odom_msg_.twist.twist.linear.z = state_est.v(2);

    odom_msg_.twist.twist.angular.x = state_est.w(0);
    odom_msg_.twist.twist.angular.y = state_est.w(1);
    odom_msg_.twist.twist.angular.z = state_est.w(2);

    odometry_pub_.publish(odom_msg_);

    // Pub Euler Attitude
    euler_msg_.header = msg->header;
    const Eigen::Vector3d euler_angles = state_est.q.euler() * 180. / M_PI;
    euler_msg_.vector.x = euler_angles(0);
    euler_msg_.vector.y = euler_angles(1);
    euler_msg_.vector.z = euler_angles(2);

    euler_deg_pub_.publish(euler_msg_);

    // Pub Imu Bias estimate
    imu_bias_msg_.header = msg->header;

    imu_bias_msg_.angular_velocity.x = state_est.bg(0);
    imu_bias_msg_.angular_velocity.y = state_est.bg(1);
    imu_bias_msg_.angular_velocity.z = state_est.bg(2);

    imu_bias_msg_.linear_acceleration.x = state_est.ba(0);
    imu_bias_msg_.linear_acceleration.y = state_est.ba(1);
    imu_bias_msg_.linear_acceleration.z = state_est.ba(2);

    // linear acceleration covariance
    imu_bias_msg_.linear_acceleration_covariance[0] = imu_R_(0,0);
    imu_bias_msg_.linear_acceleration_covariance[1] = imu_R_(0,1);
    imu_bias_msg_.linear_acceleration_covariance[2] = imu_R_(0,2);
    imu_bias_msg_.linear_acceleration_covariance[3] = imu_R_(1,0);
    imu_bias_msg_.linear_acceleration_covariance[4] = imu_R_(1,1);
    imu_bias_msg_.linear_acceleration_covariance[5] = imu_R_(1,2);
    imu_bias_msg_.linear_acceleration_covariance[6] = imu_R_(2,0);
    imu_bias_msg_.linear_acceleration_covariance[7] = imu_R_(2,1);
    imu_bias_msg_.linear_acceleration_covariance[8] = imu_R_(2,2);

    imu_bias_pub_.publish(imu_bias_msg_);

    // Only publish is_flying is true once
    if (!is_flying_) 
    {
      is_flying_ = ekf_.isFlying();
      if (is_flying_)
      {
        is_flying_msg_.data = is_flying_;
        is_flying_pub_.publish(is_flying_msg_);
      }
    }
  }
}
