/*
 * Copyright (c) 2017 Robert Leishman, BYU MAGICC Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <controller/simple_pid.h>

namespace controller
{
SimplePID::SimplePID()
{
  kp_ = 0.0;
  ki_ = 0.0;
  kd_ = 0.0;
  integrator_ = 0.0;
  differentiator_ = 0.0;
  last_error_ = 0.0;
  last_state_ = 0.0;
  tau_ = 0.0;
  max_ = DBL_MAX;
  min_ = -DBL_MAX;
}

SimplePID::SimplePID(double p, double i, double d, double max, double min, double tau) :
  kp_(p), ki_(i), kd_(d), max_(max), min_(min), tau_(tau)
{
  integrator_ = 0.0;
  differentiator_ = 0.0;
  last_error_ = 0.0;
  last_state_ = 0.0;
}

double SimplePID::computePID(double desired, double current, double dt, double x_dot)
{
    double error = desired - current;
    if (dt < 0.00001 || std::abs(error) > 9999999)
    {
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
      i_term = getIntegralTerm(dt, error);
    }

    last_error_ = error;
    last_state_ = current;

    double u = p_term + i_term - d_term;

    return compute_anti_windup(u, p_term, i_term, d_term);
}

double SimplePID::getDerivativeTerm(double current, double dt, double x_dot)
{
    double d_term;
    if (std::isfinite(x_dot))
    {
      return kd_ * x_dot;
    }
    else if (dt > 0.0)
    {
      // Noise reduction (See "Small Unmanned Aircraft". Chapter 6. Slide 31/33)
      // d/dx w.r.t. error:: differentiator_ = (2*tau_ - dt)/(2*tau_ + dt)*differentiator_ + 2/(2*tau_ + dt)*(error -
      // last_error_);
      differentiator_ =
          (2 * tau_ - dt) / (2 * tau_ + dt) * differentiator_ + 2 / (2 * tau_ + dt) * (current - last_state_);
      return kd_* differentiator_;
    }
}

double SimplePID::getIntegralTerm(double dt, double error)
{
    integrator_ += dt / 2 * (error + last_error_); // (trapezoidal rule)
    return ki_ * integrator_;
}

double SimplePID::compute_anti_windup(double u, double p_term, double i_term, double d_term)
{
    double u_sat = saturate(u, min_, max_);
    //TODO:: I switched this from "fabs(u... to fabs(u_sat..."  Need to make sure that is right.
    if (u != u_sat && std::fabs(i_term) > fabs(u_sat - p_term + d_term))
    {
      integrator_ = (u_sat - p_term + d_term) / ki_;
    }

    return u_sat;
}

void SimplePID::setGains(double p, double i, double d, double tau, double max_u, double min_u)
{
    //! \todo Should we really be zeroing this while we are gain tuning?
    kp_ = p;
    ki_ = i;
    kd_ = d;
    tau_ = tau;
    max_ = max_u;
    min_ = min_u;
}

inline double SimplePID::saturate(double val, double &min, double &max)
{
  if (val > max)
    val = max;
  else if (val < min)
    val = min;
  return val;
}

void SimplePID::clearIntegrator()
{
  integrator_ = 0.0;
}

}
