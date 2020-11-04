#ifndef ROTOR_CONTROLLER_PD_COND_I_H

#include <cmath>
// #include <ros/ros.h>  // included temporarily for debug statements

#include "controller/simple_pid.h"
#define ROTOR_CONTROLLER_PD_COND_I_H

namespace controller
{

class PDConditionalI : public controller::SimplePID
{
public:
  PDConditionalI();
  PDConditionalI(double p, double i, double d, double max, double min, double tau, double conditional_integrator_threshold);
  
  double computePDConditionalI(double desired, double current, double dt, double x_dot = INFINITY);
  void setGains(double p, double i, double d, double tau, double max_u, double min_u, double conditional_integrator_threshold);

protected:
  double getConditionalI(double dt, double error);

  bool integrator_on_{false};

  double conditional_integrator_threshold_;
};
}

#endif
