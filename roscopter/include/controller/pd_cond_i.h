#ifndef ROTOR_CONTROLLER_PD_COND_I_H
#define ROTOR_CONTROLLER_PD_COND_I_H

#include <cmath>
#include <ros/ros.h>  // included temporarily for debug statements

#include "controller/simple_pid.h"

namespace controller
{

class PDConditionalI : public controller::SimplePID
{
public:

  double computePDConditionalI(double desired, double current, double dt, double x_dot = INFINITY);

protected:
  bool integrator_on_{false};
  double conditional_integrator_threshold_{0.5};
  //TODO need to get this from ff_cntrl

  double getConditionalI(double dt, double error);
};
}

#endif
