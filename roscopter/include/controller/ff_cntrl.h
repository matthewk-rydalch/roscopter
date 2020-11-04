#ifndef FF_CNTRL_H
#define FF_CNTRL_H

#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include "controller/controller.h"
#include "controller/pd_cond_i.h"

namespace controller
{

class Ff_Cntrl : public controller::Controller
{
public:
    Ff_Cntrl();
    void computeFeedForwardControl(double dt);
    void setPDConditionalIN(double P, double I, double D, double tau);
    void setPDConditionalIE(double P, double I, double D, double tau);

    state_t target_hat_;
    bool use_feed_forward_{false};
    bool is_landing_{false};
    bool add_integrator_{false};
    bool landed_{false};

    double Kff_x_;
    double Kff_y_;
    double Kff_u_;
    double Kff_v_;
    double ramp_down_gain_;
    double conditional_integrator_threshold_;

protected:

    bool debug_Ff_Cntrl_{false};
    bool debug_computeFeedForwardControl_{false};
    bool switched_controller_{false};

    Eigen::Vector3d getBoatVelocity();

    Eigen::Matrix3d Rroll(double phi);
    Eigen::Matrix3d Rpitch(double theta);
    Eigen::Matrix3d Ryaw(double psi);

    void calcFfXposYposYawLoops(double dt);
    void calcFfXvelYvelAltLoops(double dt);

    controller::PDConditionalI pd_cond_i_n_;
    controller::PDConditionalI pd_cond_i_e_;
};
}

#endif