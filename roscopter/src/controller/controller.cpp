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
  if (debug_Controller_)
    std::cout << "In Controller::Controller!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
  // Retrieve global MAV equilibrium throttle. This is the only MAV specific
  // parameter that is required
  // ros::NodeHandle nh_mav(ros::this_node::getNamespace());
}

void Controller::computeControl(double dt)
{
  if (debug_computeControl_)
    std::cout << "In Controller::computeControl!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
  if(dt <= 0.0000001)
  {
    // This messes up the derivative calculation in the PID controllers
    return;
  }

  mode_flag_ = control_mode_;

  if(mode_flag_ == MODE_XPOS_YPOS_YAW_ALTITUDE_)
  {
    // Figure out desired velocities (in inertial frame)
    // By running the position controllers
    double pndot_c = PID_n_.computePID(xc_.pn, xhat_.pn, dt);
    double pedot_c = PID_e_.computePID(xc_.pe, xhat_.pe, dt);
    // Calculate desired yaw rate
    xc_.psi = determineShortestDirectionPsi(xc_.psi,xhat_.psi);

    xc_.r = PID_psi_.computePID(xc_.psi, xhat_.psi, dt);
    rotateVelocityCommandsToVehicle1Frame(pndot_c, pedot_c);

    mode_flag_ = MODE_XVEL_YVEL_YAWRATE_ALTITUDE_;
  }

  if(mode_flag_ == MODE_XVEL_YVEL_YAWRATE_ALTITUDE_)
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
    mode_flag_ = MODE_XACC_YACC_YAWRATE_AZ_;
  }

  if(mode_flag_ == MODE_XACC_YACC_YAWRATE_AZ_)
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
   
    mode_flag_ = MODE_ROLL_PITCH_YAWRATE_THROTTLE_;
  }

  if(mode_flag_ == MODE_ROLL_PITCH_YAWRATE_THROTTLE_)
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
  if (debug_resetIntegrators_)
    std::cout << "In Controller::resetIntegrators!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
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
  if (debug_saturate_)
    std::cout << "In Controller::saturate!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
  x = (x > max) ? max : x;
  x = (x < min) ? min : x;
  return x;
}

double Controller::determineShortestDirectionPsi(double psi_c, double psi_hat)
{
    if(fabs(psi_c + 2*M_PI - psi_hat) < fabs(psi_c - psi_hat))
    {
      psi_c += 2*M_PI;
    }
    else if (fabs(psi_c - 2*M_PI -psi_hat) < fabs(psi_c - psi_hat))
    {
      psi_c -= 2*M_PI;
    }
    return psi_c;
}

void Controller::rotateVelocityCommandsToVehicle1Frame(double pndot_c, double pedot_c)
{
    xc_.x_dot = pndot_c*cos(xhat_.psi) + pedot_c*sin(xhat_.psi);
    xc_.y_dot = -pndot_c*sin(xhat_.psi) + pedot_c*cos(xhat_.psi);
}

void Controller::setPIDXDot(double P, double I, double D, double tau)
{
  if (debug_setGains_)
    std::cout << "In Controller::setPIDXDot!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
  PID_x_dot_.setGains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);
}
void Controller::setPIDYDot(double P, double I, double D, double tau)
{
  if (debug_setGains_)
    std::cout << "In Controller::setPIDYDot!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
  PID_y_dot_.setGains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);
}
void Controller::setPIDZDot(double P, double I, double D, double tau)
{
  if (debug_setGains_)
    std::cout << "In Controller::setPIDZDot!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
  // set max z accelerations so that we can't fall faster than 1 gravity
  PID_z_dot_.setGains(P, I, D, tau, 1.0, -max_accel_z_);
}
void Controller::setPIDN(double P, double I, double D, double tau)
{
  if (debug_setGains_)
    std::cout << "In Controller::setPIDN!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
  PID_n_.setGains(P, I, D, tau, max_.n_dot, -max_.n_dot);
}
void Controller::setPIDE(double P, double I, double D, double tau)
{
  if (debug_setGains_)
    std::cout << "In Controller::setPIDE!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
  PID_e_.setGains(P, I, D, tau, max_.e_dot, -max_.e_dot);
}
void Controller::setPIDD(double P, double I, double D, double tau)
{
  if (debug_setGains_)
    std::cout << "In Controller::setPIDD!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
  PID_d_.setGains(P, I, D, tau, max_.d_dot, -max_.d_dot);
}
void Controller::setPIDPsi(double P, double I, double D, double tau)
{
  if (debug_setGains_)
    std::cout << "In Controller::setPIDPsi!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
  PID_psi_.setGains(P, I, D, tau);
}
}