#ifndef VEL_CNTRL_H
#define VEL_CNTRL_H

#include <iostream>
#include <math.h>
#include <Eigen/Dense>

#include "controller/controller.h"

namespace controller
{

class Vel_Cntrl : public controller::Controller
{
public:
    Vel_Cntrl();
    void computeVelocityControl(double dt);

    state_t target_hat_;
    bool use_feed_forward_{false};
    bool is_landing_{false};
    bool landed_{false};

protected:

    double velocityModel(double xc, double xhat, double Km);
    double Km_n_{1.0};
    double Km_e_{1.0};
    double Km_psi_{1.0};

    bool debug_Vel_Cntrl_{false};
    bool debug_computeVelocityControl_{false};
    bool debug_velocityModel_{false};

    void addFeedForwardTerm();

    Eigen::Matrix3d Rroll(double phi);
    Eigen::Matrix3d Rpitch(double theta);
    Eigen::Matrix3d Ryaw(double psi);

};
}

#endif