#include <controller/pd_i_lpf.h>

namespace controller
{
//
// Basic initialization
//
PdILpf::PdILpf()
{
  kp_ = 0.0;
  ki_ = 0.0;
  kd_ = 0.0;
  integrator_ = 0.0;
  differentiator_ = 0.0;
  last_error_ = 0.0;
  last_error_lpf_ = 0.0;
  last_state_ = 0.0;
  tau_ = 0.0;
  sigma_ = 0.0;
  max_ = DBL_MAX;
  min_ = -DBL_MAX;
}

//
// Initialize the controller
//
PdILpf::PdILpf(double p, double i, double d, double max, double min, double tau, double sigma) :
  kp_(p), ki_(i), kd_(d), max_(max), min_(min), tau_(tau), sigma_(sigma)
{
  integrator_ = 0.0;
  differentiator_ = 0.0;
  last_error_ = 0.0;
  last_error_lpf_ = 0.0;
  last_state_ = 0.0;
}

//
// Compute the control;
//
double PdILpf::computePdILpf(double desired, double current, double dt, double x_dot)
{
  double error = desired - current;

  // Don't do stupid things (like divide by nearly zero, gigantic control jumps)
  if (dt < 0.00001 || std::abs(error) > 9999999)
  {
    return 0.0;
  }

  if (dt > 1.0)
  {
    // This means that this is a ''stale'' controller and needs to be reset.
    // This would happen if we have been operating in a different mode for a while
    // and will result in some enormous integrator.
    // Or, it means we are disarmed and shouldn't integrate
    // Setting dt for this loop will mean that the integrator and dirty derivative
    // doesn't do anything this time but will keep it from exploding.
    dt = 0.0;
    differentiator_ = 0.0;
  }

  double p_term = error*kp_;
  double i_term = 0.0;
  double d_term = 0.0;

  // Calculate Derivative Term
  if (kd_ > 0.0)
  {
    if (std::isfinite(x_dot))
    {
      d_term = kd_ * x_dot;
    }
    else if (dt > 0.0)
    {
      // Noise reduction (See "Small Unmanned Aircraft". Chapter 6. Slide 31/33)
      // d/dx w.r.t. error:: differentiator_ = (2*tau_ - dt)/(2*tau_ + dt)*differentiator_ + 2/(2*tau_ + dt)*(error -
      // last_error_);
      differentiator_ =
          (2 * tau_ - dt) / (2 * tau_ + dt) * differentiator_ + 2 / (2 * tau_ + dt) * (current - last_state_);
      d_term = kd_* differentiator_;
    }
  }

  // Calculate Integrator Term
  if (ki_ > 0.0)
  {
    double error_lpf = lowPassFilter(error, dt);
    integrator_ += dt / 2 * (error_lpf + last_error_lpf_); // (trapezoidal rule)
    i_term = ki_ * integrator_;
  }

  // Save off this state for next loop
  last_error_ = error;
  last_state_ = current;

  // Sum three terms
  double u = p_term + i_term - d_term;

  // return u;

  // Integrator anti-windup
  double u_sat = saturate(u, min_, max_);
  if (u != u_sat && std::fabs(i_term) > fabs(u - p_term + d_term))
  {
    // If we are at the saturation limits, then make sure the integrator doesn't get
    // bigger if it won't do anything (except take longer to unwind).  Just set it to the
    // largest value it could be to max out the control
    integrator_ = (u_sat - p_term + d_term) / ki_;
  }
  return u_sat;
}

double PdILpf::lowPassFilter(double error, double dt)
{
  double error_lpf = error*dt/(sigma_+dt) + last_error_lpf_*sigma_/(sigma_+dt);
  return error_lpf;
}

//
// Late initialization or redo
//
void PdILpf::setGains(double p, double i, double d, double tau, double sigma, double max_u, double min_u)
{
  //! \todo Should we really be zeroing this while we are gain tuning?
  kp_ = p;
  ki_ = i;
  kd_ = d;
  tau_ = tau;
  sigma_ = sigma;
  max_ = max_u;
  min_ = min_u;
}


}  // controller
