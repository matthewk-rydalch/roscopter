#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <tf/tf.h>
#include <stdint.h>
#include <controller/simple_pid.h>
#include "roscopter_utils/yaml.h"

namespace controller
{

typedef struct
{
  double pn;
  double pe;
  double pd;

  double phi;
  double theta;
  double psi;

  double u;
  double v;
  double w;

  double p;
  double q;
  double r;
} state_t;

typedef struct
{
  double pn;
  double pe;
  double pd;

  double phi;
  double theta;
  double psi;

  double x_dot;
  double y_dot;
  double z_dot;

  double r;

  double ax;
  double ay;
  double az;

  double throttle;
} command_t;

typedef struct
{
  double roll;
  double pitch;
  double yaw_rate;
  double throttle;
  double n_dot;
  double e_dot;
  double d_dot;
} max_t;

class Controller
{
public:

  Controller();

  uint8_t MODE_PASS_THROUGH_;
  uint8_t MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE_;
  uint8_t MODE_ROLL_PITCH_YAWRATE_THROTTLE_;
  uint8_t MODE_ROLL_PITCH_YAWRATE_ALTITUDE_;
  uint8_t MODE_XPOS_YPOS_YAW_ALTITUDE_;
  uint8_t MODE_XVEL_YVEL_YAWRATE_ALTITUDE_;
  uint8_t MODE_XACC_YACC_YAWRATE_AZ_;

  state_t xhat_ = {};
  command_t xc_ = {};
  max_t max_ = {};
  double min_altitude_;
  double throttle_eq_;
  double max_accel_xy_;
  double max_accel_z_;
  uint8_t mode_flag_;
  uint8_t control_mode_;

  void computeControl(double dt);
  void resetIntegrators();

  void setPIDXDot(double P, double I, double D, double tau);
  void setPIDYDot(double P, double I, double D, double tau);
  void setPIDZDot(double P, double I, double D, double tau);
  void setPIDN(double P, double I, double D, double tau);
  void setPIDE(double P, double I, double D, double tau);
  void setPIDD(double P, double I, double D, double tau);
  void setPIDPsi(double P, double I, double D, double tau);

protected:

  double mass_;
  double max_thrust_;

  float throttle_down_ = 0.95;

  bool debug_Controller_{false};
  bool debug_computeControl_{false};
  bool debug_resetIntegrators_{false};
  bool debug_saturate_{false};
  bool debug_setGains_{false};

  // PID Controllers
  controller::SimplePID PID_x_dot_;
  controller::SimplePID PID_y_dot_;
  controller::SimplePID PID_z_dot_;
  controller::SimplePID PID_n_;
  controller::SimplePID PID_e_;
  controller::SimplePID PID_d_;
  controller::SimplePID PID_psi_;

  double saturate(double x, double max, double min);
  double determineShortestDirectionPsi(double psi_c, double psi_hat);
  void rotateVelocityCommandsToVehicle1Frame(double pndot_c, double pedot_c);

  void calcXposYposYawLoops(double dt);
  void calcXvelYvelAltLoops(double dt);
  void calcXaccYaccAzLoops(double dt);

};
} //namespace controller

#endif
