#ifndef FF_CNTRL_H
#define FF_CNTRL_H

#include <iostream>
#include <math.h>
#include <Eigen/Dense>

#include "controller/controller.h"

namespace controller
{

class Ff_Cntrl : public controller::Controller
{
public:
    Ff_Cntrl();
    void computeFeedForwardControl(double dt);

    state_t target_hat_;
    bool use_feed_forward_{false};
    bool is_landing_{false};
    bool landed_{false};

protected:

    double velocityModel(double xc, double xhat, double Km);
    double Km_n_{2.0};
    double Km_e_{2.0};
    double Km_psi_{1.0};

    bool debug_Ff_Cntrl_{false};
    bool debug_computeFeedForwardControl_{false};
    bool debug_velocityModel_{false};

    void addFeedForwardTerm();

    Eigen::Matrix3d Rroll(double phi);
    Eigen::Matrix3d Rpitch(double theta);
    Eigen::Matrix3d Ryaw(double psi);

};
}

#endif