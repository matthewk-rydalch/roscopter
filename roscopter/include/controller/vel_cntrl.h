#ifndef VEL_CNTRL_H
#define VEL_CNTRL_H

#include <iostream>

#include "controller/controller.h"

namespace controller
{

class Vel_Cntrl : public controller::Controller
{
public:
    Vel_Cntrl();
    void computeVelocityControl(double dt);

protected:

    double velocityModel(double xc, double xhat, double Km);
    double Km_n_{3.0};
    double Km_e_{3.0};
    double Km_psi_{3.0};

    bool debug_Vel_Cntrl_{false};
    bool debug_computeVelocityControl_{false};
    bool debug_velocityModel_{false};
};
}

#endif