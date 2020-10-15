#include <controller/controller.h>
#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <fstream>

namespace controller
{
Controller::Controller() //:
  // nh_(ros::NodeHandle()),
  // nh_private_("~")
{
  std::cout << "in controller \n";
  // Retrieve global MAV equilibrium throttle. This is the only MAV specific
  // parameter that is required
  // ros::NodeHandle nh_mav(ros::this_node::getNamespace());

  // Calculate max accelerations. Assuming that equilibrium throttle produces
  // 1 g of acceleration and a linear thrust model, these max acceleration
  // values are computed in g's as well.
  max_accel_z_ = 1.0 / throttle_eq_;
  max_accel_xy_ = sin(acos(throttle_eq_)) / throttle_eq_ / sqrt(2.);

  is_flying_ = false;
  received_cmd_ = false;
}

void Controller::load(const std::string &filename)
{
  // parameter that is required
  if (!roscopter::get_yaml_node("equilibrium_throttle", filename, throttle_eq_))
    ROS_ERROR("[Controller] MAV equilibrium_throttle not found!");

  roscopter::get_yaml_node("max_roll", filename, max_.roll);
  roscopter::get_yaml_node("max_pitch", filename, max_.pitch);
  roscopter::get_yaml_node("max_yaw_rate", filename, max_.yaw_rate);
  roscopter::get_yaml_node("max_throttle", filename, max_.throttle);
  roscopter::get_yaml_node("max_n_dot", filename, max_.n_dot);
  roscopter::get_yaml_node("max_e_dot", filename, max_.e_dot);
  roscopter::get_yaml_node("max_d_dot", filename, max_.d_dot);

  roscopter::get_yaml_node("min_altitude", filename, min_altitude_);
}

void Controller::computeControl(double dt)
{
  if(dt <= 0.0000001)
  {
    // This messes up the derivative calculation in the PID controllers
    return;
  }

  uint8_t mode_flag = control_mode_;

  if(mode_flag == rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE)
  {
    // Figure out desired velocities (in inertial frame)
    // By running the position controllers
    double pndot_c = PID_n_.computePID(xc_.pn, xhat_.pn, dt);
    double pedot_c = PID_e_.computePID(xc_.pe, xhat_.pe, dt);
    // Calculate desired yaw rate
    // First, determine the shortest direction to the commanded psi
    if(fabs(xc_.psi + 2*M_PI - xhat_.psi) < fabs(xc_.psi - xhat_.psi))
    {
      xc_.psi += 2*M_PI;
    }
    else if (fabs(xc_.psi - 2*M_PI -xhat_.psi) < fabs(xc_.psi - xhat_.psi))
    {
      xc_.psi -= 2*M_PI;
    }
    xc_.r = PID_psi_.computePID(xc_.psi, xhat_.psi, dt);

    xc_.x_dot = pndot_c*cos(xhat_.psi) + pedot_c*sin(xhat_.psi);
    xc_.y_dot = -pndot_c*sin(xhat_.psi) + pedot_c*cos(xhat_.psi);

    mode_flag = rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE;
  }

  if(mode_flag == rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE)
  {
    // Rotate body frame velocities to vehicle 1 frame velocities
    double sinp = sin(xhat_.phi);
    double cosp = cos(xhat_.phi);
    double sint = sin(xhat_.theta);
    double cost = cos(xhat_.theta);
    double pxdot =
        cost * xhat_.u + sinp * sint * xhat_.v + cosp * sint * xhat_.w;
    double pydot = cosp * xhat_.v - sinp * xhat_.w;
    double pzdot =
        -sint * xhat_.u + sinp * cost * xhat_.v + cosp * cost * xhat_.w;

    // TODO: Rotate boat body frame velocities into drone vehicle 1 frame velocities
    // Compute desired accelerations (in terms of g's) in the vehicle 1 frame
    xc_.z_dot = PID_d_.computePID(xc_.pd, xhat_.pd, dt, pzdot);

    xc_.ax = PID_x_dot_.computePID(xc_.x_dot, pxdot, dt);
    xc_.ay = PID_y_dot_.computePID(xc_.y_dot, pydot, dt);

    // Nested Loop for Altitude
    xc_.az = PID_z_dot_.computePID(xc_.z_dot, pzdot, dt);
    mode_flag = rosflight_msgs::Command::MODE_XACC_YACC_YAWRATE_AZ;
  }

  if(mode_flag == rosflight_msgs::Command::MODE_XACC_YACC_YAWRATE_AZ)
  {
    // Model inversion (m[ax;ay;az] = m[0;0;g] + R'[0;0;-T]
    double total_acc_c = sqrt((1.0 - xc_.az) * (1.0 - xc_.az) +
                              xc_.ax * xc_.ax + xc_.ay * xc_.ay);  // (in g's)
    if (total_acc_c > 0.001)
    {
      xc_.phi = asin(xc_.ay / total_acc_c);
      xc_.theta = -1.0*asin(xc_.ax / total_acc_c);
    }
    else
    {
      xc_.phi = 0;
      xc_.theta = 0;
    }

    // Compute desired thrust based on current pose
    double cosp = cos(xhat_.phi);
    double cost = cos(xhat_.theta);
    xc_.throttle = (1.0 - xc_.az) * throttle_eq_ / cosp / cost;

    mode_flag = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  }

  if(mode_flag == rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE)
  {
    // Pack up and send the command

    xc_.throttle = saturate(xc_.throttle, max_.throttle, 0.0);
    xc_.phi = saturate(xc_.phi, max_.roll, -max_.roll);
    xc_.theta = saturate(xc_.theta, max_.pitch, -max_.pitch);
    xc_.r = saturate(xc_.r, max_.yaw_rate, -max_.yaw_rate);
    if (-xhat_.pd < min_altitude_)
    {
      xc_.phi = 0.;
      xc_.theta = 0.;
      xc_.r = 0.;
    }
  }
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

double Controller::saturate(double x, double max, double min)
{
  x = (x > max) ? max : x;
  x = (x < min) ? min : x;
  return x;
}

void Controller::setPIDXDot(double P, double I, double D, double tau)
{
  PID_x_dot_.setGains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);
}
void Controller::setPIDYDot(double P, double I, double D, double tau)
{
  PID_y_dot_.setGains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);
}
void Controller::setPIDZDot(double P, double I, double D, double tau)
{
  // set max z accelerations so that we can't fall faster than 1 gravity
  PID_z_dot_.setGains(P, I, D, tau, 1.0, -max_accel_z_);
}
void Controller::setPIDN(double P, double I, double D, double tau)
{
  PID_n_.setGains(P, I, D, tau, max_.n_dot, -max_.n_dot);
}
void Controller::setPIDE(double P, double I, double D, double tau)
{
  PID_e_.setGains(P, I, D, tau, max_.e_dot, -max_.e_dot);
}
void Controller::setPIDD(double P, double I, double D, double tau)
{
  PID_d_.setGains(P, I, D, tau, max_.d_dot, -max_.d_dot);
}
void Controller::setPIDPsi(double P, double I, double D, double tau)
{
  PID_psi_.setGains(P, I, D, tau);
}
}