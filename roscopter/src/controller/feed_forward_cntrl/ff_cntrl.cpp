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
    return;
  }

  if (is_landing_)
  {
    if (xc_.throttle <= 0.1)	
    {  	
      xc_.throttle = 0.0;	
    }	
    else	
    {	
      xc_.throttle = ramp_down_gain_*xc_.throttle;	
    }	
  }
  else
  {  
    mode_flag_ = control_mode_;
    if(mode_flag_ == MODE_XPOS_YPOS_YAW_ALTITUDE_)
    {
      calcFfXposYposYawLoops(dt);
      mode_flag_ = MODE_XVEL_YVEL_YAWRATE_ALTITUDE_;
    }
    if(mode_flag_ == MODE_XVEL_YVEL_YAWRATE_ALTITUDE_)
    {
      if (use_feed_forward_)
        calcFfXvelYvelAltLoops(dt);
      else
        calcXvelYvelAltLoops(dt);
      mode_flag_ = MODE_XACC_YACC_YAWRATE_AZ_;
    }
    if(mode_flag_ == MODE_XACC_YACC_YAWRATE_AZ_)
    {
      calcXaccYaccAzLoops(dt);
      mode_flag_ = MODE_ROLL_PITCH_YAWRATE_THROTTLE_;
    }
    if(mode_flag_ == MODE_ROLL_PITCH_YAWRATE_THROTTLE_)
    {
      xc_.throttle = saturate(xc_.throttle, max_.throttle, 0.0);
      xc_.phi = saturate(xc_.phi, max_.roll, -max_.roll);
      xc_.theta = saturate(xc_.theta, max_.pitch, -max_.pitch);
      xc_.r = saturate(xc_.r, max_.yaw_rate, -max_.yaw_rate);
      // if (-xhat_.pd < min_altitude_)
      // {
      //   xc_.phi = 0.;
      //   xc_.theta = 0.;
      //   xc_.r = 0.;
      // }
    }
  }
}

void Ff_Cntrl::calcFfXposYposYawLoops(double dt)
{
    double pndot_c = pd_cond_i_n_.computePDConditionalI(xc_.pn, xhat_.pn, dt);
    double pedot_c = pd_cond_i_e_.computePDConditionalI(xc_.pe, xhat_.pe, dt);
    xc_.psi = determineShortestDirectionPsi(xc_.psi,xhat_.psi);
    xc_.r = PID_psi_.computePID(xc_.psi, xhat_.psi, dt);
    rotateVelocityCommandsToVehicle1Frame(pndot_c, pedot_c);

    if(use_feed_forward_) //Is this needed?  We don't enter the function unless this is true anyway.
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
    if(use_feed_forward_) //This may be causing some instability.
    {
      Eigen::Vector3d base_velocity_rover_v1_frame{getBoatVelocity()};
      xc_.ax += Kff_u_*base_velocity_rover_v1_frame[0];
      xc_.ay += Kff_v_*base_velocity_rover_v1_frame[1];
      xc_.z_dot += Kff_w_*base_velocity_rover_v1_frame[2]; //Just added this in
      // xc_.ax += Kff_u_*xc_.x_dot;
      // xc_.ay += Kff_v_*xc_.y_dot;
    }

    // Nested Loop for Altitude
    xc_.az = PID_z_dot_.computePID(xc_.z_dot, pzdot, dt);
}

Eigen::Vector3d Ff_Cntrl::getBoatVelocity()
{
  Eigen::Matrix3d Rphi = Ff_Cntrl::Rroll(target_hat_.phi);
  Eigen::Matrix3d Rth = Ff_Cntrl::Rpitch(target_hat_.theta);
  Eigen::Matrix3d Rpsi = Ff_Cntrl::Ryaw(target_hat_.psi - xhat_.psi);

  Eigen::Vector3d base_velocity_body_frame(target_hat_.u, target_hat_.v, target_hat_.w);
  Eigen::Vector3d base_velocity_rover_v1_frame(Rpsi*Rth*Rphi*base_velocity_body_frame);

  return base_velocity_rover_v1_frame;
}

void Ff_Cntrl::setPDConditionalIN(double P, double I, double D, double tau)
{
  pd_cond_i_n_.setGains(P, I, D, tau, max_.n_dot, -max_.n_dot, conditional_integrator_threshold_);
}

void Ff_Cntrl::setPDConditionalIE(double P, double I, double D, double tau)
{
  pd_cond_i_e_.setGains(P, I, D, tau, max_.e_dot, -max_.e_dot, conditional_integrator_threshold_);
}

double Ff_Cntrl::getIntegrator(bool get_x_not_y)
{
  if (get_x_not_y)
  {
    return pd_cond_i_n_.getIntegrator();
  }
  else
  {
    return pd_cond_i_e_.getIntegrator();
  }
  
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