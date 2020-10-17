#ifndef VEL_CNTRL_H
#define VEL_CNTRL_H

#include <iostream>

#include "controller/controller.h"

class Vel_Cntrl : public controller::Controller
{
public:
    Vel_Cntrl();
    
    void computeVelocityCommand();

    controller::command_t xc_;
    controller::state_t xhat_;
    uint8_t mode_flag_;

protected:

    double velocityModel(double xc, double xhat, double Km);
    double determineShortestDirectionPsi(double psi_c, double psi_hat);
    void rotateVelocityCommandsToVehicle1Frame(double pndot_c, double pedot_c);

    double Km_n_;
    double Km_e_;
    double Km_psi_;
    uint8_t MODE_XVEL_YVEL_YAWRATE_ALTITUDE_ = 5;
};

#endif