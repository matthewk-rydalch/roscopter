#include <controller/pd_cond_i.h>

namespace controller
{

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
    if (last_error_ < 0)
    {
      if (error < last_error_+conditional_integrator_threshold_)
        integrator_on_ = true;
      else
        integrator_on_ = false;
    }
    else
    {
      if (error > last_error_ - conditional_integrator_threshold_)
        integrator_on_ = true;
      else
        integrator_on_ = false;
    }

    if (integrator_on_)
      integrator_ += dt / 2 * (error + last_error_); // (trapezoidal rule)
    return ki_ * integrator_;
}

}