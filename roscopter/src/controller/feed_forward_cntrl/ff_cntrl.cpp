#include "controller/ff_cntrl.h"

namespace controller
{
Ff_Cntrl::Ff_Cntrl()
{
  //TODO: clear integrator
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
  mode_flag_ = control_mode_;
  if(mode_flag_ == MODE_XPOS_YPOS_YAW_ALTITUDE_)
  {
    calcFfXposYposYawLoops(dt);
    mode_flag_ = MODE_XVEL_YVEL_YAWRATE_ALTITUDE_;
  }

  if(use_feed_forward_)
  {
    // if (is_landing_)
    // {
    //   if (xc_.throttle <= 0.1)
    //   {  
    //     xc_.throttle = 0.0;
    //   }
    //   else
    //   {
    //     xc_.throttle = ramp_down_term_*xc_.throttle;
    //   }
    // }
    // else
    // {

      if(mode_flag_ == MODE_XVEL_YVEL_YAWRATE_ALTITUDE_)
      {
        calcFfXvelYvelAltLoops(dt);
        mode_flag_ = MODE_XACC_YACC_YAWRATE_AZ_;
        computeControl(dt);
      }
      else
      {
        computeControl(dt); 
      }
    // }
  }
  else
  {
    computeControl(dt);
  }
}

void Ff_Cntrl::calcFfXposYposYawLoops(double dt)
{
    // double pndot_c = PID_n_.computePID(xc_.pn, xhat_.pn, dt);
    double pedot_c = PID_e_.computePID(xc_.pe, xhat_.pe, dt);
    double pndot_c = pd_cond_i_n_.computePDConditionalI(xc_.pn, xhat_.pn, dt);
    // double pedot_c = pd_cond_i_e_.computePDConditionalI(xc_.pe, xhat_.pe, dt);
    xc_.psi = determineShortestDirectionPsi(xc_.psi,xhat_.psi);
    xc_.r = PID_psi_.computePID(xc_.psi, xhat_.psi, dt);
    rotateVelocityCommandsToVehicle1Frame(pndot_c, pedot_c);

    if(use_feed_forward_)
    {
      Eigen::Vector3d base_velocity_rover_v1_frame{getBoatVelocity()};
      xc_.x_dot += Kff_x_*base_velocity_rover_v1_frame[0];
      xc_.y_dot -= Kff_y_*base_velocity_rover_v1_frame[1];//TODO why does this have to be negative?  Frames?
    }
}

void Ff_Cntrl::calcFfXvelYvelAltLoops(double dt)
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
    if(use_feed_forward_)
    {
      xc_.ax += Kff_u_*xc_.x_dot;
      xc_.ay += Kff_v_*xc_.y_dot;
    }

    // Nested Loop for Altitude
    xc_.az = PID_z_dot_.computePID(xc_.z_dot, pzdot, dt);
}

Eigen::Vector3d Ff_Cntrl::getBoatVelocity()
{
  Eigen::Matrix3d Rphi = Ff_Cntrl::Rroll(target_hat_.phi);
  Eigen::Matrix3d Rth = Ff_Cntrl::Rpitch(target_hat_.theta);
  Eigen::Matrix3d Rpsi = Ff_Cntrl::Ryaw(target_hat_.psi + xhat_.psi);

  Eigen::Vector3d base_velocity_body_frame(target_hat_.u, target_hat_.v, target_hat_.w);
  Eigen::Vector3d base_velocity_rover_v1_frame(Rpsi*Rth*Rphi*base_velocity_body_frame);

  return base_velocity_rover_v1_frame;
}

void Ff_Cntrl::setPDCondIGains(double Pn, double In, double Dn, double Pe, double Ie, double De, double tau)
{
    pd_cond_i_n_.setGains(Pn, In, Dn, max_.n_dot, -max_.n_dot, tau);
    pd_cond_i_e_.setGains(Pe, Ie, De, max_.n_dot, -max_.n_dot, tau);
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