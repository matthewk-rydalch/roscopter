#ifndef PD_I_LPF_H
#define PD_I_LPF_H

#include <cmath>
#include <ros/ros.h>  // included temporarily for debug statements

namespace controller
{
class PdILpf
{
public:
  PdILpf();

  /*!
   * \brief SimplePID initializes the class.
   * \param p the proportional controller gain (required)
   * \param i the integral controller gain (defaults to zero)
   * \param d the derivative controller gain (defaults to zero)
   * \param imin the min value accepted in the output of the integral control
   * \param imax the max value accepted in the output of the integral control (saturation for integrator windup)
   * \param tau band limited differentiator to reduce noise
   * \param sigma parameter for low pass filter on error for integrator (reject high frequency content)
   */
  PdILpf(double p, double i = 0.0, double d = 0.0, double max = DBL_MAX, double min = -DBL_MAX, double tau = 0.15, double sigma = 0.3);

  /*!
   * \brief computePID computes the PID control for the given error and timestep (since the last control was computed!)
   * \param p_error is the "position" error (or whatever variable you are controlling)
   * \param dt is the timestep since the last control was computed.
   * \param x_dot derivative of current state (optional)
   * \return the control command
   */
  double computePdILpf(double desired, double current, double dt, double x_dot = INFINITY);

  /*!
   * \brief setgains is used to set the gains for a controller after it's been initialized.  It will rewrite
   *  whatever is already there!
   * \param p the proportional controller gain (required)
   * \param i the integral controller gain (defaults to zero)
   * \param d the derivative controller gain (defaults to zero)
   * \param tau band limited differentiator to reduce noise
   * \param sigma parameter for low pass filter on error for integrator (reject high frequency content)
   */
  void setGains(double p, double i = 0.0, double d = 0.0, double tau = 0.15, double sigma = 0.3, double max_u = DBL_MAX, double min_u = -DBL_MAX);

  /*!
   * \brief setgains is used to set the gains for a controller after it's been initialized.  It will rewrite
   *  whatever is already there!
   * \param max the largest output allowed (integrator anti-windup will kick in at this value as well)
   * \param min the smallest output allowed (also activates integrator anti-windup
   */
  void setLimits(double max, double min);

  /*!
   * \brief clearIntegrator allows you to clear the integrator, in case of integrator windup.
   */

  double lowPassFilter(double error, double dt);

  void clearIntegrator()
  {
    integrator_ = 0.0;
  }

protected:
  double kp_;              //!< the proportional gain
  double ki_;              //!< the integral gain (zero if you don't want integral control)
  double kd_;              //!< the derivative gain (zero if you don't want derivative control)
  double integrator_;      //!< the integral of p_error
  double differentiator_;  //!< used for noise reduced differentiation
  double last_error_;      //!< the last p_error, for computing the derivative;
  double last_error_lpf_;
  double last_state_;      //!< the last state, for computing the derivative;
  double tau_;             //!< the noise reduction term for the derivative
  double sigma_;           //!< the noise reduction term for the integrator
  double max_;             //!< Maximum Output
  double min_;             //!< Minimum Output

  /*!
   * \brief saturate saturates the variable val
   * \param val the parameter to saturate (makes a copy)
   * \param min the minimum value
   * \param max the max value
   * \return the saturated (if necessary) value
   */
  inline double saturate(double val, double &min, double &max)
  {
    if (val > max)
      val = max;
    else if (val < min)
      val = min;
    return val;
  }
};
}

#endif