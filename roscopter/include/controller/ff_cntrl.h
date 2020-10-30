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
    void setPDCondIGains(double Pn, double In, double Dn, double Pe, double Ie, double De, double tau);

    state_t target_hat_;
    bool use_feed_forward_{false};
    bool is_landing_{false};
    bool add_integrator_{false};
    bool landed_{false};

protected:
    // double Kff_x_{1.0};
    // double Kff_y_{1.0};
    // double Kff_u_{0.07};
    // double Kff_v_{0.07};
      double Kff_x_{0.0};
    double Kff_y_{0.0};
    double Kff_u_{0.0};
    double Kff_v_{0.0};
    double ramp_down_term_{0.90};

    bool debug_Ff_Cntrl_{false};
    bool debug_computeFeedForwardControl_{false};
    bool switched_controller_{false};

    Eigen::Vector3d getBoatVelocity();

    Eigen::Matrix3d Rroll(double phi);
    Eigen::Matrix3d Rpitch(double theta);
    Eigen::Matrix3d Ryaw(double psi);

    void calcFfXposYposYawLoops(double dt);
    void calcFfXvelYvelAltLoops(double dt);

      // PID Controllers
    controller::PDConditionalI pd_cond_i_n_;
    controller::PDConditionalI pd_cond_i_e_;
};
}

#endif