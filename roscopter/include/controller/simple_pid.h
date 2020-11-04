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

/*!
 *  \brief This file defines a simple PID controller to be used by other classes to implement a PID control loop
 *  \author Robert Leishman
 *  \date Dec. 2013
*/

#ifndef ROTOR_CONTROLLER_SIMPLE_PID_H
#define ROTOR_CONTROLLER_SIMPLE_PID_H

#include <cmath>
#include <ros/ros.h>  // included temporarily for debug statements

namespace controller
{

class SimplePID
{
public:
  SimplePID();
  SimplePID(double p, double i = 0.0, double d = 0.0, double max = DBL_MAX, double min = -DBL_MAX, double tau = 0.15);

  double computePID(double desired, double current, double dt, double x_dot = INFINITY);
  void setGains(double p, double i = 0.0, double d = 0.0, double tau = 0.15, double max_u = DBL_MAX, double min_u = -DBL_MAX);
  void setLimits(double max, double min);
  void clearIntegrator();

protected:
  double kp_;              //!< the proportional gain
  double ki_;              //!< the integral gain (zero if you don't want integral control)
  double kd_;              //!< the derivative gain (zero if you don't want derivative control)
  double integrator_;      //!< the integral of p_error
  double differentiator_;  //!< used for noise reduced differentiation
  double last_error_;      //!< the last p_error, for computing the derivative;
  double last_state_;      //!< the last state, for computing the derivative;
  double tau_;             //!< the noise reduction term for the derivative
  double max_;             //!< Maximum Output
  double min_;             //!< Minimum Output

  double getDerivativeTerm(double current, double dt, double xdot);
  double getIntegralTerm(double dt, double error);
  double compute_anti_windup(double u, double p_term, double i_term, double d_term);
  inline double saturate(double val, double &min, double &max);
};
}

#endif
