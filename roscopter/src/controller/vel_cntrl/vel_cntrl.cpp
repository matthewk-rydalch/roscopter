#include "controller/vel_cntrl.h"

Vel_Cntrl::Vel_Cntrl()
{}

void Vel_Cntrl::computeVelocityCommand()
{
    double pndot_c = velocityModel(xc_.pn,xhat_.pn,Km_n_);
    double pedot_c = velocityModel(xc_.pe,xhat_.pe,Km_e_);
    xc_.psi = determineShortestDirectionPsi(xc_.psi,xhat_.psi);
    xc_.r = velocityModel(xc_.psi,xhat_.psi,Km_psi_);
    rotateVelocityCommandsToVehicle1Frame(pndot_c, pedot_c);
    mode_flag_ = MODE_XVEL_YVEL_YAWRATE_ALTITUDE_;
}

double Vel_Cntrl::velocityModel(double xc, double xhat, double Km)
{
    double velocity_command = Km*pow(xc-xhat,3);
    return velocity_command;
}

double Vel_Cntrl::determineShortestDirectionPsi(double psi_c, double psi_hat)
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

void Vel_Cntrl::rotateVelocityCommandsToVehicle1Frame(double pndot_c, double pedot_c)
{
    xc_.x_dot = pndot_c*cos(xhat_.psi) + pedot_c*sin(xhat_.psi);
    xc_.y_dot = -pndot_c*sin(xhat_.psi) + pedot_c*cos(xhat_.psi);
}