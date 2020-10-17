#include "controller/vel_cntrl.h"

namespace controller
{
Vel_Cntrl::Vel_Cntrl()
{}

void Vel_Cntrl::computeVelocityControl(double dt)
{
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
    mode_flag_ = MODE_XVEL_YVEL_YAWRATE_ALTITUDE_;
    computeControl(dt);
  }
  else
  {
    computeControl(dt);
  }
}

double Vel_Cntrl::velocityModel(double xc, double xhat, double Km)
{
    double velocity_command = Km*pow(xc-xhat,3);
    return velocity_command;
}
}