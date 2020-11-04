#include <controller/pd_cond_i.h>

namespace controller
{
PDConditionalI::PDConditionalI() :
  SimplePID()
{}

PDConditionalI::PDConditionalI(double p, double i, double d, double max, double min, double tau, double conditional_integrator_threshold) :
  SimplePID(p, i, d, max, min, tau), conditional_integrator_threshold_(conditional_integrator_threshold)
{
  integrator_ = 0.0;
  differentiator_ = 0.0;
  last_error_ = 0.0;
  last_state_ = 0.0;
}

double PDConditionalI::computePDConditionalI(double desired, double current, double dt, double x_dot)
{
    double error = desired - current;

    if (dt < 0.00001 || std::abs(error) > 9999999)
    {
      std::cout << "dt small or error big";
      return 0.0;
    }

    if (dt > 1.0)
    {
      dt = 0.0;
      differentiator_ = 0.0;
    }

    double p_term = error*kp_;
    double i_term = 0.0;
    double d_term = 0.0;

    if (kd_ > 0.0)
    {
      d_term = getDerivativeTerm(current, dt, x_dot);
    }

    if (ki_ > 0.0)
    {
      i_term = getConditionalI(dt, error);
    }

    last_error_ = error;
    last_state_ = current;

    double u = p_term + i_term - d_term;

    // if integrator_on_ ?????
    return compute_anti_windup(u, p_term, i_term, d_term);;
}

double PDConditionalI::getConditionalI(double dt, double error)
{
    double de_dt = (abs(error)-abs(last_error_))/dt;

    if (de_dt > -conditional_integrator_threshold_)
      integrator_on_ = true;
    else
      integrator_on_ = false;

    if (integrator_on_)
      integrator_ += dt / 2 * (error + last_error_); // (trapezoidal rule)
    return ki_ * integrator_;
}

void PDConditionalI::setGains(double p, double i, double d, double tau, double max_u, double min_u, double conditional_integrator_threshold)
{
    //! \todo Should we really be zeroing this while we are gain tuning?
    kp_ = p;
    ki_ = i;
    kd_ = d;
    tau_ = tau;
    max_ = max_u;
    min_ = min_u;
    conditional_integrator_threshold_ = conditional_integrator_threshold;
}

}