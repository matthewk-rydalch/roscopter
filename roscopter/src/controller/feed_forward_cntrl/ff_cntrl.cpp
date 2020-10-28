#include "controller/ff_cntrl.h"

namespace controller
{
Ff_Cntrl::Ff_Cntrl()
{
    if (debug_Ff_Cntrl_)
      std::cout << "In Ff_Contrl::Ff_Cntrl!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
}

void Ff_Cntrl::computeFeedForwardControl(double dt)
{
  if (debug_computeFeedForwardControl_)
    std::cout << "In Ff_Cntrl::computeFeedForwardControl!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
  if(dt <= 0.0000001)
  {
    // This messes up the derivative calculation in the PID controllers
    return;
  }

  if(control_mode_ == MODE_XPOS_YPOS_YAW_ALTITUDE_)
  {
    double pndot_c = velocityModel(xc_.pn,xhat_.pn,Km_n_);
    double pedot_c = velocityModel(xc_.pe,xhat_.pe,Km_e_);
    xc_.psi = determineShortestDirectionPsi(xc_.psi,xhat_.psi);
    xc_.r = velocityModel(xc_.psi,xhat_.psi,Km_psi_);
    rotateVelocityCommandsToVehicle1Frame(pndot_c, pedot_c);
    if(use_feed_forward_)
    {
      addFeedForwardTerm();
    }
    control_mode_ = MODE_XVEL_YVEL_YAWRATE_ALTITUDE_;
    computeControl(dt);
    control_mode_ = MODE_XPOS_YPOS_YAW_ALTITUDE_;
  }
  else
  {
    computeControl(dt);
  }
}

double Ff_Cntrl::velocityModel(double xc, double xhat, double Km)
{
  if (debug_velocityModel_)
    std::cout << "In Ff_Contrl::velocityModel!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
  double velocity_command = Km*atan(xc-xhat);
  return velocity_command;
}

void Ff_Cntrl::addFeedForwardTerm()
{

  Eigen::Matrix3d Rphi = Ff_Cntrl::Rroll(target_hat_.phi);
  Eigen::Matrix3d Rth = Ff_Cntrl::Rpitch(target_hat_.theta);
  Eigen::Matrix3d Rpsi = Ff_Cntrl::Ryaw(target_hat_.psi + xhat_.psi);

  Eigen::Vector3d base_velocity_body_frame(target_hat_.u, target_hat_.v, target_hat_.w);

  Eigen::Vector3d base_velocity_rover_v1_frame(Rpsi*Rth*Rphi*base_velocity_body_frame);

  xc_.x_dot += base_velocity_rover_v1_frame[0]; //feed forward the base velocity
  xc_.y_dot -= base_velocity_rover_v1_frame[1]; //TODO why does this have to be negative?  Frames?
}

Eigen::Matrix3d Ff_Cntrl::Rroll(double phi)
{
  double cp = cos(phi);
  double sp = sin(phi);
  Eigen::Matrix3d Rphi;
  Rphi << 1.0, 0.0, 0.0,
          0.0,  cp, -sp,
          0.0,  sp,  cp;
  return Rphi;
}

Eigen::Matrix3d Ff_Cntrl::Rpitch(double theta)
{
  double ct = cos(theta);
  double st = sin(theta);
  Eigen::Matrix3d Rth;
  Rth <<  ct, 0.0,  st,
          0.0, 1.0, 0.0,
          -st, 0.0,  ct;
  return Rth;
}

Eigen::Matrix3d Ff_Cntrl::Ryaw(double psi)
{
  double cp = cos(psi);
  double sp = sin(psi);
  Eigen::Matrix3d Rpsi;
  Rpsi <<  cp, -sp, 0.0,
           sp,  cp, 0.0,
          0.0, 0.0, 1.0;

  return Rpsi;
}
}