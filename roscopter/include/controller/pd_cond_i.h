#ifndef ROTOR_CONTROLLER_PD_COND_I_H
#define ROTOR_CONTROLLER_PD_COND_I_H
#include <cmath>
#include "controller/simple_pid.h"

namespace controller
{

class PDConditionalI : public controller::SimplePID
{
public:
  PDConditionalI();
  PDConditionalI(double p, double i, double d, double max, double min, double tau, double conditional_integrator_threshold);

  double computePDConditionalI(double desired, double current, double dt, double x_dot = INFINITY);
  void setGains(double p, double i, double d, double tau, double max_u, double min_u, double conditional_integrator_threshold);
  double getIntegrator();

protected:
  double getConditionalI(double dt, double error);

  bool integrator_on_{false};

  double conditional_integrator_threshold_;
};
}

#endif
